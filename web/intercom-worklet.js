/* Mono PCM16 voice transport. Resampling uses the actual AudioContext rate. */
class IntercomProcessor extends AudioWorkletProcessor {
  constructor(options) {
    super();this.rate=options.processorOptions.rate;this.input=[];this.inputAt=0;
    this.frame=new Int16Array(this.rate/50);this.frameAt=0;this.output=[];this.outputAt=0;
    this.microphone=true;this.speaker=true;
    this.port.onmessage=event=>{
      if (event.data.play) {
        const pcm=new Int16Array(event.data.play);
        if (this.output.length>this.rate*.3) {this.output=[];this.outputAt=0;}
        for (const value of pcm) this.output.push(value/32768);
      }
      if (event.data.microphone!==undefined) this.microphone=event.data.microphone;
      if (event.data.speaker!==undefined) this.speaker=event.data.speaker;
    };
  }
  process(inputs,outputs) {
    const input=inputs[0]?.[0], output=outputs[0]?.[0];
    if (input) {
      for (const value of input) this.input.push(value);
      while (this.inputAt+1<this.input.length) {
        const at=Math.floor(this.inputAt),fraction=this.inputAt-at;
        const value=this.microphone?this.input[at]*(1-fraction)+this.input[at+1]*fraction:0;
        this.frame[this.frameAt++]=Math.max(-32768,Math.min(32767,Math.round(value*32767)));
        this.inputAt+=sampleRate/this.rate;
        if (this.frameAt===this.frame.length) {
          this.port.postMessage(this.frame.buffer,[this.frame.buffer]);this.frame=new Int16Array(this.rate/50);this.frameAt=0;
        }
      }
      const consumed=Math.floor(this.inputAt);this.input.splice(0,consumed);this.inputAt-=consumed;
    }
    if (output) {
      for (let index=0;index<output.length;index++) {
        const at=Math.floor(this.outputAt),fraction=this.outputAt-at;
        if (at+1<this.output.length) {
          output[index]=this.speaker?(this.output[at]*(1-fraction)+this.output[at+1]*fraction):0;
          this.outputAt+=this.rate/sampleRate;
        } else output[index]=0;
      }
      const consumed=Math.floor(this.outputAt);this.output.splice(0,consumed);this.outputAt-=consumed;
    }
    return true;
  }
}
registerProcessor('robot-intercom',IntercomProcessor);
