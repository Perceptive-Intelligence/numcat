const mqtt = require('mqtt');
const fs = require('fs');
const readline = require('readline');

//重试机制
let retryCount = 0; // 当前重试次数
const maxRetries = 5; // 最大重试次数
let retryInterval = null; // 定时器变量，用于重试


// MQTT server details
const mqttServerUrl = 'mqtt://60.204.251.153';
//const mqttServerUrl = 'mqtt://192.168.152.2';
const firmwareTopic = 'device/firmware';  // Firmware topic
const commandTopic = 'device/command';    // Command topic
const debugTopic = 'device/debug';        // Debug topic for updates and confirmations
const  dataTopic = "device/ele_run";      // 数据上传主题

const clientId = 'ESP32Client_01';        // Client ID
const firmwarePath = "C://Users//01//Documents//Arduino//TAN//TAN.ino.esp32c3.bin";  // Firmware path

// Create MQTT client
const client = mqtt.connect(mqttServerUrl, { clientId });


//const chunkSize = 32768;  // 32KB per chunk
const chunkSize = 8192;


// 补齐设备 ID 的函数
function completeDeviceId(deviceId) {
    if (!deviceId.startsWith("ESP32_") && deviceId !== 'ALL') {
        return `ESP32_${deviceId}`;
    }
    return deviceId;
}

// Read firmware and send it in chunks (no JSON, raw binary)
function sendFirmwareInChunks(deviceId, firmwareSize) {
    fs.readFile(firmwarePath, (err, data) => {
        if (err) {
            console.error('Failed to read firmware file:', err);
            return;
        }

        console.log(`Firmware file size: ${data.length} bytes`);  // Print total firmware size
        
        let offset = 0;
        while (offset < data.length) {
            const chunk = data.slice(offset, offset + chunkSize);

            // Log the chunk being sent
            console.log(`Sending chunk from ${offset} to ${offset + chunk.length} bytes.`);

            // Send binary chunk to firmwareTopic
            client.publish(firmwareTopic, chunk, { qos: 1 }, (err) => {
                if (err) {
                    console.error('Error sending firmware chunk:', err);
                } else {
                    console.log(`Successfully sent chunk from ${offset} to ${offset + chunk.length} bytes.`);
                }
            });

            offset += chunkSize;
        }

        console.log('Firmware transmission complete.');
    });
}

// User input interface
let rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

// State to track if the update process has been confirmed
let updatePending = false;
let currentDevice = null;

// Send firmware size to the device(s)
function sendFirmwareSize(deviceId, firmwareSize) {
    client.publish(commandTopic, JSON.stringify({
        deviceId: deviceId,
        command: '0x01',
        size: firmwareSize
    }), { qos: 1 });
}
function retryOTAUpdate() {
    if (retryCount < maxRetries && updatePending) {
        console.log(`等待 2 秒没有收到确认信息，正在重试... (${retryCount + 1}/${maxRetries})`);
        // 再次发送 OTA 指令
        fs.stat(firmwarePath, (err, stats) => {
            if (err) {
                console.error('获取固件大小失败:', err);
            } else {
                sendFirmwareSize(currentDevice, stats.size);
            }
        });
        retryCount++;
    } else if (retryCount >= maxRetries) {
        console.error('重试次数已达到最大限制，OTA 更新失败');
        clearInterval(retryInterval); // 停止重试
    }
}


// Initial user input to choose between no update, WiFi update, or OTA update
rl.question('请选择操作 (0: 不更新, 1: OTA 更新, 2: WiFi 更新): ', (input) => {
    if (input === '0') {
        console.log('不进行任何更新。');
        rl.close();
    } else if (input === '1') {
        rl.question('选择设备进行 OTA 更新 (输入 ALL 进行一键更新，或输入设备ID，例如 001, 056, 100): ', (deviceId) => {
            currentDevice = completeDeviceId(deviceId.trim());  // 不补齐 ALL


            console.log('发送 OTA 更新指令...');
            updatePending = true;
            // 发送固件大小给设备
            fs.stat(firmwarePath, (err, stats) => {
                if (err) {
                    console.error('获取固件大小失败:', err);
                    rl.close();
                } else {
                    sendFirmwareSize(currentDevice, stats.size);
                }
            });
            // 启动每2秒的重试机制
            retryInterval = setInterval(retryOTAUpdate, 2000);
            rl.close();
        });
    } else if (input === '2') {
        console.log('WiFi 更新即将开始...');
        client.publish(commandTopic, JSON.stringify({ deviceId: 'ESP32_008', command: '0x02' }));
        rl.close();
    } else {
        console.log('无效的输入。');
        rl.close();
    }
});
//bug,要按好几次服务器才可以实现OTA更新，加一个逻辑，按下1之后监听是否有返回信息和接收，2S之类没有继续发指令
// Listen for incoming messages
client.on('message', (topic, message) => {
    console.log(`收到消息来自主题 ${topic}: ${message.toString()}`);

    // If OTA update confirmed by the device
    if (topic === debugTopic && message.toString() === '0x01' && updatePending) {
        console.log('收到 OTA 更新确认消息，准备执行更新...');
        fs.stat(firmwarePath, (err, stats) => {
            if (err) {
                console.error('获取固件大小失败:', err);
            } else {
                sendFirmwareInChunks(currentDevice, stats.size);
            }
        });
        updatePending = false;
        clearInterval(retryInterval); // 收到确认消息，停止重试

    }

    // 处理设备完成的OTA更新确认
    if (topic === debugTopic && message.toString().includes('OTA 更新完成')) {
        console.log(`设备已完成OTA更新: ${message.toString()}`);
    }


        // 处理 ele_run 主题，打印电压数据
    if (topic === dataTopic) {
        console.log(`Client ID: ${clientId}, Received message on topic ${topic}: ${message.toString()}`);
        }
});
// Subscribe to topics once connected
client.on('connect', () => {
    client.subscribe([commandTopic, debugTopic, dataTopic], (err) => {  // 添加 dataTopic
        if (err) {
            console.error('Failed to subscribe to topics:', err);
        } else {
            console.log('Subscribed to command, debug, and data topics');  // 订阅成功时的反馈
        }
    });
});





