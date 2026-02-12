// SPDX-License-Identifier: LicenseRef-Proprietary

let ros = null;
let pubIntent = null;
let subTask = null;
let subAudit = null;

function logLine(s) {
  const out = document.getElementById("out");
  out.textContent = (s + "\n" + out.textContent).slice(0, 12000);
}

function setStatus(s) {
  document.getElementById("status").textContent = s;
}

function connect() {
  const url = document.getElementById("ws").value.trim();
  if (!url) return;

  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    setStatus("Connected: " + url);

    pubIntent = new ROSLIB.Topic({
      ros,
      name: "/fo/hmi/intent_text",
      messageType: "std_msgs/String",
    });

    subTask = new ROSLIB.Topic({
      ros,
      name: "/fo/task",
      messageType: "std_msgs/String",
    });
    subTask.subscribe((msg) => logLine("[TASK] " + msg.data));

    subAudit = new ROSLIB.Topic({
      ros,
      name: "/fo/audit",
      messageType: "std_msgs/String",
    });
    subAudit.subscribe((msg) => logLine("[AUDIT] " + msg.data));
  });

  ros.on("error", (e) => setStatus("Error: " + e));
  ros.on("close", () => setStatus("Closed."));
}

function sendIntent() {
  const text = document.getElementById("intent").value.trim();
  if (!text || !pubIntent) return;

  pubIntent.publish(new ROSLIB.Message({ data: text }));
  logLine("[HMI] Sent intent: " + text);
}

function clearIntent() {
  document.getElementById("intent").value = "";
}

document.getElementById("connect").addEventListener("click", connect);
document.getElementById("send").addEventListener("click", sendIntent);
document.getElementById("clear").addEventListener("click", clearIntent);
