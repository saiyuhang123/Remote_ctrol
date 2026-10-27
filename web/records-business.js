(() => {
  'use strict';
  const el=id=>document.getElementById(id), esc=escapeHtml;
  const state={tab:'',reportOffset:0,resultOffset:0,cursor:'',visionPages:0,visionBusy:false,revision:0,
    reportGeneration:0,reportOpenGeneration:0,resultGeneration:0,visionGeneration:0,captureGeneration:0,visionController:null,runOptionsLoaded:false};
  const sections={
    reports:{panel:'reports',title:'巡检报告',hint:'按任务查看完整报告，查看照片并下载 HTML 或 JSON。'},
    results:{panel:'recognition',title:'识别记录',kind:'',hint:'查看到点抓拍、仪表识别和全部视觉记录；可按关联任务筛选。'},
    faces:{panel:'recognition',title:'人脸抓拍记录',kind:'face_capture',hint:'查看人脸照片、记录时间和小车位置；点击缩略图打开原图。'},
    persons:{panel:'recognition',title:'行人检测记录',kind:'person',hint:'查看行人检测照片、记录时间和小车位置。'},
    plates:{panel:'recognition',title:'车牌识别记录',kind:'vehicle_plate',hint:'查看识别到的车牌号码、照片、时间和小车位置。'},
    details:{panel:'details',title:'巡检明细',hint:'按巡检任务汇总日期、路线、人脸和行人次数及车牌统计。'},
    captures:{panel:'captures',title:'手动抓拍记录',hint:'查看网页独立手动抓拍的照片，点击缩略图打开原图。'},
  };
  const tabs=[...document.querySelectorAll('[data-record-tab]')];
  function resetDetails() {
    state.revision++; state.resultGeneration++; state.visionGeneration++;
    state.visionController?.abort(); state.visionController=null; state.visionBusy=false;
    state.resultOffset=0; state.cursor=''; state.visionPages=0;
    el('visionGallery').textContent='正在读取记录…'; el('visionCount').textContent=''; el('visionMore').disabled=true;
    el('inspectionRows').innerHTML='<tr><td colspan="5" class="muted">正在读取到点识别记录…</td></tr>';
    el('resultPage').textContent=''; el('resultPrev').disabled=el('resultNext').disabled=true;
  }
  function selectTab(name, reset=false) {
    if (!Object.prototype.hasOwnProperty.call(sections,name)) name='reports';
    if (state.tab===name && !reset) return;
    state.tab=name; resetDetails();
    const section=sections[name];
    for (const button of tabs) {
      const selected=button.dataset.recordTab===name;
      button.setAttribute('aria-selected',String(selected)); button.tabIndex=selected?0:-1;
      if (selected) el('recordsRecognition').setAttribute('aria-labelledby',button.id);
    }
    for (const panel of document.querySelectorAll('[data-record-panel]')) panel.hidden=panel.dataset.recordPanel!==section.panel;
    el('recordsSectionHint').textContent=section.hint;
    el('recognitionTitle').textContent=section.title;
    el('inspectionSection').hidden=name!=='results'; el('visionTitle').hidden=name!=='results';
    try { sessionStorage.setItem('robot-records-section',name); } catch (_) { /* Browsers may disable storage. */ }
    window.scrollTo(0,0);
    refreshCurrent();
  }
  for (const [index,button] of tabs.entries()) {
    button.onclick=()=>selectTab(button.dataset.recordTab);
    button.onkeydown=event=>{
      let next;
      if (event.key==='ArrowRight') next=(index+1)%tabs.length;
      else if (event.key==='ArrowLeft') next=(index+tabs.length-1)%tabs.length;
      else if (event.key==='Home') next=0;
      else if (event.key==='End') next=tabs.length-1;
      else return;
      event.preventDefault(); tabs[next].focus(); selectTab(tabs[next].dataset.recordTab);
      tabs[next].scrollIntoView({block:'nearest',inline:'nearest'});
    };
  }
  let captureOffset=0;
  async function loadCaptures() {
    const generation=++state.captureGeneration;
    const response=await fetch(`/api/records/captures?limit=30&offset=${captureOffset}`,{cache:'no-store'}),data=await response.json();
    if (!response.ok) throw new Error(data.detail || '抓拍记录读取失败');
    if (generation!==state.captureGeneration) return;
    el('captureGallery').replaceChildren();
    for (const item of data.items || []) {
      const article=document.createElement('article');article.className='vision-event';
      const name=document.createElement('strong');name.textContent=item.label;
      const at=document.createElement('small');at.textContent=new Date(item.stamp*1000).toLocaleString('zh-CN');
      if (item.available && /^\/captures\/[0-9-]+\/[\w.-]+\.jpg$/.test(item.image_url)) {
        const link=document.createElement('a'),picture=document.createElement('img');link.href=picture.src=item.image_url;link.target='_blank';link.rel='noopener';picture.loading='lazy';picture.alt='手动抓拍';link.append(picture);article.append(link);
      } else {const missing=document.createElement('small');missing.textContent='图片已过期或被清理';article.append(missing);}
      article.append(name,at);el('captureGallery').append(article);
    }
    el('captureCount').textContent=`共 ${data.total} 张 · 第 ${Math.floor(captureOffset/30)+1} 页`;
    el('capturesPrev').disabled=captureOffset===0;el('capturesNext').disabled=captureOffset+(data.items?.length || 0)>=data.total;
  }
  for (const [id,offset] of [['capturesPrev',-30],['capturesNext',30],['refreshCaptures',0]]) el(id).onclick=()=>{captureOffset=Math.max(0,captureOffset+offset);refreshCurrent();};
  const names={face_capture:'人脸抓拍',person:'行人',vehicle_plate:'车牌'};
  const statuses={9:'已取消',10:'已完成',11:'失败',12:'已中断'};
  const date=value=>value?new Date(value/1e6).toLocaleString('zh-CN',{hour12:false}):'—';
  const run=()=>el('recordRun').value;
  const error=value=>{el('businessError').textContent=value;};
  const media=url=>typeof url==='string' && /^\/api\/(vision\/media|inspection\/results)\//.test(url)?url:'';
  const image=url=>media(url)?`<a href="${esc(url)}" target="_blank" rel="noopener"><img src="${esc(url)}" loading="lazy" alt="识别抓拍"></a>`:'<span class="muted">无图片</span>';
  async function get(url, options={}) {
    const response=await fetch(url,{cache:'no-store',...options}),data=await response.json();
    if (!response.ok) throw new Error(typeof data.detail==='string'?data.detail:'记录读取失败');
    return data;
  }
  async function loadReports() {
    const generation=++state.reportGeneration;
    const data=await get(`/api/inspection/reports?limit=20&offset=${state.reportOffset}`);
    if (generation!==state.reportGeneration) return;
    state.runOptionsLoaded=true;
    const rows=el('reportRows'); rows.replaceChildren();
    for (const item of data.items || []) {
      const tr=document.createElement('tr'),id=encodeURIComponent(item.run_id);
      tr.innerHTML=`<td>${esc(date(item.started_at))}</td><td>${esc(item.route_name)}<small class="muted report-id">${esc(item.run_id)}</small></td>
        <td>${esc(statuses[item.state] || item.state)}</td><td>${Number(item.completed_steps)}</td><td><button class="btn gray">查看报告</button>
        <a class="btn gray" href="/api/inspection/reports/${id}/download?format=html" download>下载 HTML</a>
        <a class="btn gray" href="/api/inspection/reports/${id}/download?format=json" download>下载 JSON</a></td>`;
      tr.querySelector('button').onclick=()=>openReport(item.run_id).catch(problem=>error(problem.message)); rows.append(tr);
      if (![...el('recordRun').options].some(option=>option.value===item.run_id))
        el('recordRun').add(new Option(`${date(item.started_at)} · ${item.route_name}`,item.run_id));
    }
    if (!data.items?.length) rows.innerHTML='<tr><td colspan="5" class="muted">任务结束后将在这里生成巡检报告</td></tr>';
    el('reportPage').textContent=`共 ${data.total} 份报告 · 第 ${Math.floor(state.reportOffset/20)+1} 页`;
    el('reportPrev').disabled=state.reportOffset===0;
    el('reportNext').disabled=state.reportOffset+(data.items?.length || 0)>=data.total;
  }
  async function openReport(runId) {
    const revision=state.revision,generation=++state.reportOpenGeneration;
    const data=await get(`/api/inspection/reports/${encodeURIComponent(runId)}`), reportRun=data.run;
    if (state.tab!=='reports' || revision!==state.revision || generation!==state.reportOpenGeneration) return;
    el('reportTitle').textContent=data.route.name || '巡检报告';
    const summary=data.summary;
    el('reportBody').innerHTML=`<p>${esc(data.state_name)} · ${esc(data.map_name)} · ${Number(data.duration_seconds).toFixed(1)} 秒</p>
      <p>${esc(date(reportRun.started_at))} — ${esc(date(reportRun.finished_at))}</p>
      <p>完成 ${Number(reportRun.completed_steps)} 个点 · 抓拍识别成功 ${Number(data.inspection_success_count)} 次 / 失败 ${Number(data.inspection_failure_count)} 次</p>
      <p>人脸 ${Number(summary.face_capture_count)} 次 · 行人 ${Number(summary.person_count)} 次 · 车牌 ${esc((summary.plates || []).map(item=>item.plate).join('、') || '无')}</p>
      <p>${esc(reportRun.error_code || '')} ${esc(reportRun.message || '')}</p>
      <div class="control-buttons"><a class="btn" href="/api/inspection/reports/${encodeURIComponent(runId)}/download?format=html" download>下载报告</a><button class="btn gray" id="filterReportRun">查看本次全部识别记录</button></div>
      <h3>到点抓拍与识别</h3><div class="report-inspections"></div>
      <h3>到点与停车</h3><div class="table-wrap"><table><thead><tr><th>圈 / 趟</th><th>点序</th><th>状态</th><th>位置误差</th><th>异常</th></tr></thead><tbody id="reportStops"></tbody></table></div>
      <p class="muted">关联视觉识别记录 ${Number(data.vision_events.length)} 条。HTML 报告可在浏览器中打印为 PDF。</p>`;
    const items=el('reportBody').querySelector('.report-inspections');
    for (const item of data.inspection_results) {
      const article=document.createElement('article'); article.className='report-inspection';
      article.innerHTML=`${image(item.image_url)}<div><strong>${esc(item.point_name || item.point_id)} · 第 ${Number(item.loop_index)} 圈 / 趟</strong>
        <p class="recognition-text">${esc(item.text || item.message || '')}</p><small>${item.success?'成功':esc(item.error_code || '失败')}</small></div>`;
      items.append(article);
    }
    if (!data.inspection_results.length) items.textContent='本任务没有到点抓拍识别结果。';
    for (const item of data.stop_attempts) {
      const tr=document.createElement('tr');
      tr.innerHTML=`<td>${Number(item.loop_index)}</td><td>${Number(item.step_index)}</td><td>${esc(item.result)}</td>
        <td>${item.actual_valid?Number(item.position_error).toFixed(3)+' m':'无实测'}</td><td>${esc(item.error_code || '—')}</td>`;
      el('reportStops').append(tr);
    }
    el('filterReportRun').onclick=()=>{
      if (![...el('recordRun').options].some(option=>option.value===runId)) el('recordRun').add(new Option(data.route.name,runId));
      el('recordRun').value=runId; el('reportDialog').close(); selectTab('results',true);
    };
    if (!el('reportDialog').open) el('reportDialog').showModal();
  }
  async function loadResults() {
    const revision=state.revision,generation=++state.resultGeneration;
    const data=await get(`/api/inspection/results?limit=30&offset=${state.resultOffset}&run_id=${encodeURIComponent(run())}`);
    if (revision!==state.revision || generation!==state.resultGeneration) return;
    el('inspectionRows').replaceChildren();
    for (const item of data.items || []) {
      const tr=document.createElement('tr');
      tr.innerHTML=`<td>${esc(date(item.started_at))}</td><td>${esc(item.point_name || item.point_id)}</td><td class="record-image">${image(item.image_url)}</td>
        <td class="recognition-text">${esc(item.text || item.message || '')}</td><td>${item.success?'成功':esc(item.error_code || '失败')}</td>`;
      el('inspectionRows').append(tr);
    }
    if (!data.items?.length) el('inspectionRows').innerHTML='<tr><td colspan="5" class="muted">暂无到点抓拍识别结果</td></tr>';
    el('resultPage').textContent=`共 ${data.total} 条 · 第 ${Math.floor(state.resultOffset/30)+1} 页`;
    el('resultPrev').disabled=state.resultOffset===0; el('resultNext').disabled=state.resultOffset+(data.items?.length || 0)>=data.total;
  }
  async function loadVision(more=false) {
    if (more && state.visionBusy) return;
    if (!more) state.visionController?.abort();
    const controller=new AbortController(); state.visionController=controller; state.visionBusy=true;
    const revision=state.revision,generation=++state.visionGeneration;
    const kind=sections[state.tab]?.kind || '';
    el('visionMore').disabled=true;
    try {
      const data=await get(`/api/vision/events?limit=30&event_type=${encodeURIComponent(kind)}&run_id=${encodeURIComponent(run())}&cursor=${encodeURIComponent(more?state.cursor:'')}`,{signal:controller.signal});
      if (revision!==state.revision || generation!==state.visionGeneration) return;
      if (!more) { el('visionGallery').replaceChildren(); state.visionPages=0; }
      state.visionPages++; state.cursor=data.next_cursor || '';
      for (const item of data.items || []) {
        const article=document.createElement('article'); article.className='vision-event';
        const pose=item.pose_valid?`${Number(item.robot_x).toFixed(2)}, ${Number(item.robot_y).toFixed(2)} m · ${(Number(item.robot_yaw)*180/Math.PI).toFixed(1)}°`:'无定位';
        article.innerHTML=`${image(item.image_url)}<strong>${esc(names[item.event_type] || item.event_type)} · ${esc(item.label)}</strong>
          <small>${esc(date(item.event_time_ns))}</small><small>地图：${esc(item.map_id || '未关联')}</small><small>位置：${esc(pose)}</small>`;
        el('visionGallery').append(article);
      }
      if (!data.total) el('visionGallery').textContent=`暂无${names[kind] || '视觉识别'}记录，视觉执勤产生的记录将自动上传。`;
      el('visionCount').textContent=`共 ${data.total} 条识别记录`; el('visionMore').disabled=!state.cursor;
    } catch(problem) {
      if (problem.name!=='AbortError') {
        if (generation===state.visionGeneration && !state.visionPages)
          el('visionGallery').textContent='记录读取失败，请点击“刷新当前内容”重试。';
        throw problem;
      }
    } finally {
      if (generation===state.visionGeneration) {
        state.visionBusy=false; state.visionController=null; el('visionMore').disabled=!state.cursor;
      }
    }
  }
  async function reloadDetails() {
    resetDetails(); await refreshCurrent();
  }
  async function refreshCurrent() {
    const section=state.tab,revision=state.revision;
    error('');
    try {
      if (section==='reports') await loadReports();
      else if (section==='details') { await loadRouteNames(); await load(); }
      else if (section==='captures') await loadCaptures();
      else {
        // Populate task choices independently: an unavailable patrol database
        // must not stop querying standalone face/person/plate records.
        if (!state.runOptionsLoaded) loadReports().catch(()=>{});
        const jobs=[];
        if (section==='results') jobs.push(loadResults());
        if (state.visionPages<=1) jobs.push(loadVision());
        const outcomes=await Promise.allSettled(jobs);
        const failed=outcomes.find(item=>item.status==='rejected');
        if (failed) throw failed.reason;
      }
    } catch(problem) {
      if (state.tab===section && state.revision===revision && problem.name!=='AbortError') error(problem.message);
    }
  }
  el('closeReport').onclick=()=>el('reportDialog').close();
  el('refreshBusiness').onclick=()=>{state.visionPages=0; refreshCurrent();};
  el('recordRun').onchange=()=>reloadDetails().catch(problem=>error(problem.message));
  for (const [id,key,step,load] of [['reportPrev','reportOffset',-20,loadReports],['reportNext','reportOffset',20,loadReports],
      ['resultPrev','resultOffset',-30,loadResults],['resultNext','resultOffset',30,loadResults]])
    el(id).onclick=()=>{state[key]=Math.max(0,state[key]+step); load().catch(problem=>error(problem.message));};
  el('visionMore').onclick=()=>loadVision(true).catch(problem=>error(problem.message));
  let initial='reports';
  try { initial=sessionStorage.getItem('robot-records-section') || initial; } catch (_) { /* Optional preference. */ }
  selectTab(initial); setInterval(refreshCurrent,15000);
})();
