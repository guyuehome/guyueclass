function TraceInfoFlag() {
    this.traceFlag = new Array();
    this.traceFlag["LPF.c:45c33"]=1;
    this.traceFlag["LPF.c:45c60"]=1;
    this.traceFlag["LPF.c:45c70"]=1;
    this.traceFlag["LPF.c:50c20"]=1;
    this.traceFlag["LPF.c:59c20"]=1;
    this.traceFlag["LPF.c:59c26"]=1;
    this.traceFlag["LPF.c:59c38"]=1;
    this.traceFlag["LPF.c:59c58"]=1;
}
top.TraceInfoFlag.instance = new TraceInfoFlag();
function TraceInfoLineFlag() {
    this.lineTraceFlag = new Array();
    this.lineTraceFlag["LPF.c:45"]=1;
    this.lineTraceFlag["LPF.c:46"]=1;
    this.lineTraceFlag["LPF.c:49"]=1;
    this.lineTraceFlag["LPF.c:50"]=1;
    this.lineTraceFlag["LPF.c:51"]=1;
    this.lineTraceFlag["LPF.c:53"]=1;
    this.lineTraceFlag["LPF.c:59"]=1;
    this.lineTraceFlag["LPF.c:62"]=1;
    this.lineTraceFlag["LPF.c:87"]=1;
}
top.TraceInfoLineFlag.instance = new TraceInfoLineFlag();
