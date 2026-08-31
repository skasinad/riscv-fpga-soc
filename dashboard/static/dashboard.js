const historyLength=40;

const labels=[];
const instructionHistory=[];
const dataRamHistory=[];
const mmioHistory=[];

// Index matches the WORKLOAD_ID byte in telemetry (see main.c's WL_* enum)
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


// Controls are disabled whenever a command is in flight (the FPGA only has
// one pending-response slot, see serial_reader.py's command_lock), the
// link is down, or the CPU is trapped. trapped comes from telemetry's own
// trap field, never from a button click -- a trapped PicoRV32 isn't
// running firmware, so anything that depends on firmware (workload
// selection, RESET_COUNTERS, CAPTURE_SNAPSHOT, another fault injection)
// stops making sense until SYSTEM_RESET recovers it. SYSTEM_RESET itself
// is deliberately exempt: it's the only thing that can ever get out of a
// trapped state, and it works independently of CPU execution since the
// command/response path lives below the CPU, not inside it.
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

        // This is the authoritative fault indicator -- trapped controls
        // which buttons are disabled below, and it comes from nowhere but
        // this field. Never set from a button click or an ACK.
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

        // The active workload button follows telemetry's WORKLOAD_ID, not
        // whichever button was last clicked -- this is the FPGA's own
        // reported state, so the UI can't get out of sync with hardware.
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


// One shared function for all three control requests instead of six
// near-identical fetch blocks. Controls stay disabled for the whole round
// trip (not just the click) because the FPGA can only have one command
// outstanding at a time.
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


// The ACK here only means "armed" -- it says nothing about whether the CPU
// has actually trapped yet. The #trap indicator (and everything it
// disables) only ever changes once telemetry reports trap=1 on a later
// poll, not from this response.
function injectFault() {
    sendCommand("/api/inject-fault",{},"INJECT_FAULT");
}


// Same reasoning in reverse: this ACK only means the FPGA accepted the
// reset request and the response has left the transmitter -- POST hasn't
// necessarily rerun yet. refreshTelemetry()'s normal polling is what
// eventually shows POST PASS and TRAP CLEAR again; a temporary gap in
// telemetry right after this is expected, not a disconnect.
function systemReset() {
    sendCommand("/api/system-reset",{},"SYSTEM_RESET");
}


document.querySelectorAll(".workload-btn").forEach((btn) => {
    btn.addEventListener("click",() => setWorkload(btn.dataset.workload));
});

document.getElementById("reset-counters-btn").addEventListener("click",resetCounters);
document.getElementById("snapshot-btn").addEventListener("click",captureSnapshot);
document.getElementById("inject-fault-btn").addEventListener("click",injectFault);
document.getElementById("system-reset-btn").addEventListener("click",systemReset);


setInterval(refreshTelemetry,1000);

refreshTelemetry();