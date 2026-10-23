(() => {
  'use strict';
  const el=id=>document.getElementById(id), root=window.RobotConsole;
  let ws=null, stream=null, context=null, processor=null, source=null, active=false, busy=false, generation=0, deviceWaiter=null;
  let robotReady=false;
  const message=text=>{el('intercomMessage').textContent=text;};
  function update() {
    el('intercomStart').disabled=!root.session || busy || active || !robotReady;
    el('intercomEnd').disabled=!busy && !active;
    el('intercomOpen').textContent=active?'对讲中':'麦克风 / 音响';
    for (const id of ['robotMicrophone','robotSpeaker','browserMicrophone','browserSpeaker','intercomRefresh']) el(id).disabled=active || busy;
    if (!window.AudioContext?.prototype.setSinkId) {
      el('browserSpeaker').disabled=true;el('browserSpeaker').title='当前浏览器使用系统默认音响';
    }
  }
  function clearResources() {
    if (stream) {stream.getTracks().forEach(track=>track.stop());stream=null;}
    if (source) {source.disconnect();source=null;}
    if (processor) {processor.disconnect();processor.port.onmessage=null;processor=null;}
    if (context) {context.close().catch(()=>{});context=null;}
  }
  function stop(reason='对讲已结束') {
    generation++;active=busy=false;clearResources();
    if (deviceWaiter) {deviceWaiter.reject(new Error(reason));deviceWaiter=null;}
    const previous=ws;ws=null;
    if (previous) {if (previous.readyState===1) previous.send(JSON.stringify({type:'stop'}));previous.close();}
    message(reason);update();
  }
  function populate(id,devices,type) {
    const select=el(id),previous=select.value;select.replaceChildren(new Option('系统默认',''));
    for (const device of devices.filter(item=>item.kind===type)) select.add(new Option(device.label || `${type==='audioinput'?'麦克风':'音响'} ${select.length}`,device.deviceId));
    if ([...select.options].some(option=>option.value===previous)) select.value=previous;
  }
  async function devices() {
    if (!root.session) throw new Error('请先在监控台或任务页取得网页控制权');
    if (ws) {const previous=ws;ws=null;previous.close();}
    const socket=new WebSocket(root.socketUrl+'/intercom');ws=socket;socket.binaryType='arraybuffer';
    const response=new Promise((resolve,reject)=>{deviceWaiter={resolve,reject};setTimeout(()=>{
      if (deviceWaiter && ws===socket) {deviceWaiter=null;reject(new Error('音频设备查询超时'));socket.close();}
    },10000);});
    socket.onopen=()=>socket.send(JSON.stringify({session_id:root.session}));
    socket.onmessage=event=>{
      if (ws!==socket) return;
      if (event.data instanceof ArrayBuffer) {if (active && processor) processor.port.postMessage({play:event.data},[event.data]);return;}
      let data;try {data=JSON.parse(event.data);} catch (_) {return;}
      if (data.type==='devices') {
        robotReady=false;
        for (const [id,key] of [['robotMicrophone','input_channels'],['robotSpeaker','output_channels']]) {
          const select=el(id),previous=select.value;select.replaceChildren(new Option('机器人系统默认',''));
          data.devices.filter(device=>device[key]>0).forEach(device=>select.add(new Option(device.name,String(device.id))));
          if ([...select.options].some(option=>option.value===previous)) select.value=previous;
        }
        if (!data.devices.some(device=>device.input_channels>0) || !data.devices.some(device=>device.output_channels>0)) {
          const error=new Error('机器人尚未连接麦克风或音响，接入后刷新设备。');
          stop(error.message);return;
        }
        robotReady=true;update();
        if (deviceWaiter) {deviceWaiter.resolve();deviceWaiter=null;}
      } else if (data.type==='ready') {
        if (!processor || !stream || !busy) {stop('通话已取消');return;}
        active=true;busy=false;message('双向对讲已开启：机器人麦克风 → 网页音响，网页麦克风 → 机器人音响。');update();
      } else if (data.type==='error') stop(data.message || '对讲失败');
      else if (data.type==='stopped') stop(data.message || '对讲已结束');
    };
    socket.onclose=()=>{if (ws===socket) stop('对讲连接已断开，音频设备已释放。');};
    socket.onerror=()=>{if (ws===socket) stop('对讲连接失败，请检查网关连接及 HTTPS 证书。');};
    await response;
    if (navigator.mediaDevices?.enumerateDevices) {
      const browser=await navigator.mediaDevices.enumerateDevices();
      populate('browserMicrophone',browser,'audioinput');populate('browserSpeaker',browser,'audiooutput');
    }
  }
  el('intercomOpen').onclick=()=>{el('intercomDialog').showModal();if (!active && !busy) devices().then(()=>message('机器人设备已就绪，点击开启对讲。')).catch(error=>message(error.message));};
  el('intercomClose').onclick=()=>el('intercomDialog').close();
  el('intercomRefresh').onclick=()=>devices().then(()=>message('设备列表已更新')).catch(error=>message(error.message));
  el('intercomStart').onclick=async()=>{
    if (active || busy) return;
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {message('浏览器麦克风需要 HTTPS 或 localhost；请按使用说明启用 HTTPS。');return;}
    const attempt=++generation;busy=true;update();message('正在申请网页麦克风权限并连接机器人…');
    try {
      if (!ws || ws.readyState!==1) await devices();
      if (attempt!==generation) return;
      const captured=await navigator.mediaDevices.getUserMedia({audio:{
        deviceId:el('browserMicrophone').value?{exact:el('browserMicrophone').value}:undefined,
        echoCancellation:true,noiseSuppression:true,autoGainControl:true,channelCount:1},video:false});
      if (attempt!==generation) {captured.getTracks().forEach(track=>track.stop());return;}
      stream=captured;stream.getAudioTracks().forEach(track=>{track.enabled=el('intercomMic').checked;});context=new AudioContext({latencyHint:'interactive'});
      if (el('browserSpeaker').value && context.setSinkId) await context.setSinkId(el('browserSpeaker').value);
      await context.audioWorklet.addModule('intercom-worklet.js');
      if (attempt!==generation) return;
      processor=new AudioWorkletNode(context,'robot-intercom',{numberOfInputs:1,numberOfOutputs:1,outputChannelCount:[1],processorOptions:{rate:48000}});
      processor.port.onmessage=event=>{if (active && ws?.readyState===1 && ws.bufferedAmount<19200) ws.send(event.data);};
      processor.port.postMessage({microphone:el('intercomMic').checked,speaker:el('intercomSpeaker').checked});
      source=context.createMediaStreamSource(stream);source.connect(processor);processor.connect(context.destination);await context.resume();
      if (attempt!==generation) return;
      ws.send(JSON.stringify({type:'start',input_device:el('robotMicrophone').value,output_device:el('robotSpeaker').value}));
      setTimeout(()=>{if (attempt===generation && busy) stop('开启对讲超时，设备已释放，请刷新设备后重试。');},12000);
    } catch(error) {if (attempt===generation) stop(error.name==='NotAllowedError'?'麦克风权限被拒绝，请在浏览器站点设置中允许。':error.message);}
  };
  el('intercomEnd').onclick=()=>stop();
  for (const [id,key] of [['intercomMic','microphone'],['intercomSpeaker','speaker']]) el(id).onchange=()=>{processor?.port.postMessage({[key]:el(id).checked});if (key==='microphone') stream?.getAudioTracks().forEach(track=>{track.enabled=el(id).checked;});};
  addEventListener('robot-console-state',()=>{if (!root.session && (active || busy || ws)) stop('控制权已释放，对讲已结束');update();});
  addEventListener('pagehide',()=>stop());update();
})();
