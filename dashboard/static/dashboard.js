const historyLength=40;

const labels=[];
const instructionHistory=[];
const dataRamHistory=[];
const mmioHistory=[];

//index matches the WORKLOAD_ID byte in telemetry, see main.c's WL_* enum
const WORKLOAD_NAMES=["IDLE","ALU","MEMORY","BRANCH","MMIO","MIXED"];


const chart=new Chart(
    document.getElementById("activityChart"),
    {
        type:"line",

        data:{
            labels:labels,

            datasets:[
                {
                    label:"INSTR / SEC",
                    data:instructionHistory,
                    borderWidth:1.5,
                    pointRadius:0,
                    tension:0.08
                },
                {
                    label:"DATA RAM / SEC",
                    data:dataRamHistory,
                    borderWidth:1.5,
                    pointRadius:0,
                    tension:0.08,
                    borderDash:[5,4]
                },
                {
                    label:"MMIO / SEC",
                    data:mmioHistory,
                    borderWidth:1.5,
                    pointRadius:0,
                    tension:0.08,
                    borderDash:[2,3]
                }
            ]
        },

        options:{
            responsive:true,
            maintainAspectRatio:false,
            animation:false,

            interaction:{
                intersect:false,
                mode:"index"
            },

            scales:{
                x:{
                    display:false
                },

                y:{
                    beginAtZero:true,

                    border:{
                        color:"#2b3136"
                    },

                    ticks:{
                        color:"#727c83",
                        font:{
                            family:"SFMono-Regular"
                        }
                    },

                    grid:{
                        color:"#1d2226"
                    }
                }
            },

            plugins:{
                legend:{
                    position:"bottom",

                    labels:{
                        color:"#89939a",
                        boxWidth:18,
                        boxHeight:1,
                        padding:18,
                        font:{
                            family:"SFMono-Regular",
                            size:10
                        }
                    }
                }
            }
        }
    }
);


function formatNumber(value) {
    return Math.round(value).toLocaleString();
}


//controls disable when a command is in flight, the link is down, or the
//cpu is trapped. trapped comes from telemetry's own field, never from a
//click - a trapped cpu isn't running firmware so anything that depends on
//it stops making sense until SYSTEM_RESET. reset itself stays enabled
//since it's the only way out and works independent of cpu execution
let commandBusy=false;
let linkConnected=false;
let trapped=false;

function syncControlState() {
    const disabled=commandBusy||!linkConnected;
    const disabledIfRunning=disabled||trapped;

    document.querySelectorAll(".requires-running").forEach((btn) => {
        btn.disabled=disabledIfRunning;
    });

    document.getElementById("system-reset-btn").disabled=disabled;
}


function updateHistory(data) {
    labels.push("");

    instructionHistory.push(data.instruction_rate);
    dataRamHistory.push(data.data_ram_rate);
    mmioHistory.push(data.mmio_rate);

    if(labels.length>historyLength) {
        labels.shift();
        instructionHistory.shift();
        dataRamHistory.shift();
        mmioHistory.shift();
    }

    chart.update();
}


async function refreshTelemetry() {
    try {
        const response=await fetch("/api/telemetry");
        const data=await response.json();

        const connection=document.getElementById("connection");

        linkConnected=data.connected;
        syncControlState();

        if(data.connected) {
            connection.textContent="CONNECTED";
            connection.className="link-state connected";
        }
        else {
            connection.textContent="DISCONNECTED";
            connection.className="link-state disconnected";
            return;
        }


        const post=document.getElementById("post");

        post.textContent=data.post_pass ? "PASS" : "FAIL";
        post.className=data.post_pass ? "pass" : "fail";


        const trap=document.getElementById("trap");

        //this is the real fault indicator, drives which buttons get
        //disabled below - never set from a click or an ack
        trapped=data.trap;
        syncControlState();

        trap.textContent=data.trap ? "TRAP ASSERTED" : "CLEAR";
        trap.className=data.trap ? "fail" : "pass";


        document.getElementById("cycles").textContent=
            formatNumber(data.cycles);

        document.getElementById("instructions").textContent=
            formatNumber(data.instructions);

        document.getElementById("memory").textContent=
            formatNumber(data.memory);

        document.getElementById("data-ram").textContent=
            formatNumber(data.data_ram);

        document.getElementById("mmio").textContent=
            formatNumber(data.mmio);

        document.getElementById("instruction-rate").textContent=
            formatNumber(data.instruction_rate);

        document.getElementById("memory-rate").textContent=
            formatNumber(data.memory_rate);

        document.getElementById("data-ram-rate").textContent=
            formatNumber(data.data_ram_rate);

        document.getElementById("mmio-rate").textContent=
            formatNumber(data.mmio_rate);

        document.getElementById("cpi").textContent=
            data.cpi.toFixed(2);

        document.getElementById("packets").textContent=
            data.packets.toLocaleString();

        //active workload button follows telemetry's WORKLOAD_ID, not
        //whichever button got clicked, so the ui can't drift from hardware
        const activeWorkload=WORKLOAD_NAMES[data.workload];

        document.getElementById("active-workload").textContent=
            activeWorkload||"---";

        document.querySelectorAll(".workload-btn").forEach((btn) => {
            btn.classList.toggle("active",btn.dataset.workload===activeWorkload);
        });

        updateHistory(data);
    }
    catch(error) {
        console.error(error);

        linkConnected=false;
        syncControlState();

        const connection=document.getElementById("connection");

        connection.textContent="DISCONNECTED";
        connection.className="link-state disconnected";
    }
}


//one shared function instead of six near identical fetch blocks. controls
//stay disabled for the whole round trip since the fpga only allows one
//command outstanding at a time
async function sendCommand(url,body,label) {
    if(commandBusy) {
        return null;
    }

    commandBusy=true;
    syncControlState();

    const cmdStatus=document.getElementById("cmd-status");
    cmdStatus.textContent=`${label} ...`;
    cmdStatus.className="";

    try {
        const response=await fetch(url,{
            method:"POST",
            headers:{"Content-Type":"application/json"},
            body:JSON.stringify(body||{})
        });

        const result=await response.json();

        if(result.ok) {
            cmdStatus.textContent=`${label} ${result.status}`;
            cmdStatus.className="ack";
        }
        else {
            cmdStatus.textContent=`${label} ${result.status||result.error||"FAILED"}`;
            cmdStatus.className="nack";
        }

        return result;
    }
    catch(error) {
        console.error(error);

        cmdStatus.textContent=`${label} TIMEOUT`;
        cmdStatus.className="nack";

        return null;
    }
    finally {
        commandBusy=false;
        syncControlState();
    }
}


function setWorkload(name) {
    sendCommand("/api/workload",{workload:name},`SET_WORKLOAD ${name}`);
}


function resetCounters() {
    sendCommand("/api/reset-counters",{},"RESET_COUNTERS");
}


async function captureSnapshot() {
    const result=await sendCommand("/api/snapshot",{},"CAPTURE_SNAPSHOT");

    if(result&&result.ok&&result.instruction_count!==undefined) {
        document.getElementById("snapshot-instr").textContent=
            formatNumber(result.instruction_count);
    }
}


//ack just means "armed" here, says nothing about whether the cpu has
//actually trapped yet. the #trap indicator only changes once telemetry
//reports trap=1 on a later poll, not from this response
function injectFault() {
    sendCommand("/api/inject-fault",{},"INJECT_FAULT");
}


//same idea in reverse - this ack just means the fpga accepted the reset
//request, post hasn't necessarily rerun yet. normal telemetry polling is
//what eventually shows POST PASS and TRAP CLEAR again
function systemReset() {
    sendCommand("/api/system-reset",{},"SYSTEM_RESET");
}


//event log is entirely backend owned (see serial_reader.py), this file
//just renders whatever /api/events returns, never creates one from a
//click or a guess at hardware state

//restrained text color hint, checked against message content instead of
//the backend's coarse "kind" so CONNECTED and DISCONNECTED read differently
function eventAccentClass(message) {
    if(message.includes("TRAP ASSERTED")) return "event-danger";
    if(message.includes("DISCONNECTED")) return "event-danger";
    if(message.includes("POST FAIL")) return "event-danger";
    if(message.includes("TIMEOUT")) return "event-danger";
    if(message.includes("COMMAND ERROR")) return "event-danger";
    if(message.includes("NACK")) return "event-danger";
    if(message.includes("BUSY")) return "event-warn";
    if(message.includes("ACK")) return "event-ok";
    if(message.includes("PASS")) return "event-ok";
    if(message.includes("CLEAR")) return "event-ok";
    if(message.includes("CONNECTED")) return "event-ok";
    if(message.includes("COMPLETE")) return "event-ok";
    if(message.includes("ACTIVE")) return "event-ok";
    return "";
}


//only appends new events instead of rebuilding the whole list every poll,
//avoids flicker and makes "stay pinned to bottom" simple. a shrinking
//count means the log got cleared (or flask restarted) so rebuild then
let renderedEventCount=0;

function renderEvents(events) {
    const log=document.getElementById("event-log");

    if(events.length<renderedEventCount) {
        log.innerHTML="";
        renderedEventCount=0;
    }

    if(events.length===renderedEventCount) {
        return;
    }

    const wasAtBottom=(log.scrollTop+log.clientHeight)>=(log.scrollHeight-4);

    for(let i=renderedEventCount;i<events.length;i++) {
        const event=events[i];

        const row=document.createElement("div");
        row.className="event-row";

        const time=document.createElement("span");
        time.className="event-time";
        time.textContent=event.time;

        const message=document.createElement("span");
        message.className=`event-message ${eventAccentClass(event.message)}`;
        message.textContent=event.message;

        row.appendChild(time);
        row.appendChild(message);
        log.appendChild(row);
    }

    renderedEventCount=events.length;

    if(wasAtBottom) {
        log.scrollTop=log.scrollHeight;
    }
}


async function refreshEvents() {
    try {
        const response=await fetch("/api/events");
        const data=await response.json();

        renderEvents(data.events);
    }
    catch(error) {
        console.error(error);
    }
}


async function clearEvents() {
    try {
        await fetch("/api/events/clear",{method:"POST"});

        renderedEventCount=0;
        document.getElementById("event-log").innerHTML="";
    }
    catch(error) {
        console.error(error);
    }
}


document.querySelectorAll(".workload-btn").forEach((btn) => {
    btn.addEventListener("click",() => setWorkload(btn.dataset.workload));
});

document.getElementById("reset-counters-btn").addEventListener("click",resetCounters);
document.getElementById("snapshot-btn").addEventListener("click",captureSnapshot);
document.getElementById("inject-fault-btn").addEventListener("click",injectFault);
document.getElementById("system-reset-btn").addEventListener("click",systemReset);
document.getElementById("clear-events-btn").addEventListener("click",clearEvents);


setInterval(refreshTelemetry,1000);
setInterval(refreshEvents,1000);

refreshTelemetry();
refreshEvents();