function TraceInfoFlag() {
    this.traceFlag = new Array();
    this.traceFlag["example.c:40c33"]=1;
    this.traceFlag["example.c:40c49"]=1;
    this.traceFlag["example.c:40c66"]=1;
}
top.TraceInfoFlag.instance = new TraceInfoFlag();
function TraceInfoLineFlag() {
    this.lineTraceFlag = new Array();
    this.lineTraceFlag["example.c:40"]=1;
    this.lineTraceFlag["example.h:40"]=1;
    this.lineTraceFlag["example.h:41"]=1;
    this.lineTraceFlag["example.h:42"]=1;
    this.lineTraceFlag["example.h:47"]=1;
    this.lineTraceFlag["example.h:52"]=1;
}
top.TraceInfoLineFlag.instance = new TraceInfoLineFlag();
