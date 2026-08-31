const historyLength=40;

const labels=[];
const instructionHistory=[];
const memoryHistory=[];
const mmioHistory=[];


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
                    label:"RAM / SEC",
                    data:memoryHistory,
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


function updateHistory(data) {
    labels.push("");

    instructionHistory.push(data.instruction_rate);
    memoryHistory.push(data.memory_rate);
    mmioHistory.push(data.mmio_rate);

    if(labels.length>historyLength) {
        labels.shift();
        instructionHistory.shift();
        memoryHistory.shift();
        mmioHistory.shift();
    }

    chart.update();
}


async function refreshTelemetry() {
    try {
        const response=await fetch("/api/telemetry");
        const data=await response.json();

        const connection=document.getElementById("connection");

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

        trap.textContent=data.trap ? "TRAPPED" : "CLEAR";
        trap.className=data.trap ? "fail" : "pass";


        document.getElementById("cycles").textContent=
            formatNumber(data.cycles);

        document.getElementById("instructions").textContent=
            formatNumber(data.instructions);

        document.getElementById("memory").textContent=
            formatNumber(data.memory);

        document.getElementById("mmio").textContent=
            formatNumber(data.mmio);

        document.getElementById("instruction-rate").textContent=
            formatNumber(data.instruction_rate);

        document.getElementById("memory-rate").textContent=
            formatNumber(data.memory_rate);

        document.getElementById("mmio-rate").textContent=
            formatNumber(data.mmio_rate);

        document.getElementById("cpi").textContent=
            data.cpi.toFixed(2);

        document.getElementById("packets").textContent=
            data.packets.toLocaleString();

        updateHistory(data);
    }
    catch(error) {
        console.error(error);

        const connection=document.getElementById("connection");

        connection.textContent="DISCONNECTED";
        connection.className="link-state disconnected";
    }
}


setInterval(refreshTelemetry,1000);

refreshTelemetry();