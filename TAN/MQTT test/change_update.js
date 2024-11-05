const mqtt = require('mqtt');
const readline = require('readline');

// 替换为你的 MQTT 服务器地址和端口
const mqttServerUrl = 'mqtt://60.204.251.153';
const topic = 'device/ele_run';  // 替换为ESP32发送数据的主题
const configTopic = 'device/config';  // 发送新的配置参数的主题
const configResponseTopic = 'device/config_response';  // 接收 ESP32 返回的参数
const confirmationTopic = 'device/config_confirmation';  // 确认参数的主题

// 定义客户端ID
const clientId = 'ESP32Client_01';  // 你可以根据需要设置不同的客户端ID

// 创建 MQTT 客户端，指定客户端ID
const client = mqtt.connect(mqttServerUrl, { clientId });

// 要发送的新配置参数
const newConfig = {
    ssid: 'NewSSID',
    password: 'NewPassword',
    mqtt_server: '60.204.251.154',  // 新的 MQTT 服务器地址
    deviceId: 'ESP32_009'
};

// 保存已发送的参数以进行对比
let sentConfig = JSON.stringify(newConfig);

// 创建读取用户输入的接口
const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

// 提示用户输入，控制是否发送配置
rl.question('请输入1以发送新的配置，0保持现有逻辑：', (input) => {
    if (input === '1') {
        // 当用户输入1时，发送配置
        client.on('connect', () => {
            console.log(`Connected to MQTT server with Client ID: ${clientId}`);
            
            // 订阅特定的主题
            client.subscribe(topic, (err) => {
                if (!err) {
                    console.log(`Subscribed to topic: ${topic}`);
                } else {
                    console.log(`Failed to subscribe: ${err}`);
                }
            });

            // 订阅配置响应主题
            client.subscribe(configResponseTopic, (err) => {
                if (!err) {
                    console.log(`Subscribed to topic: ${configResponseTopic}`);
                }
            });

            // 订阅确认主题
            client.subscribe(confirmationTopic, (err) => {
                if (!err) {
                    console.log(`Subscribed to topic: ${confirmationTopic}`);
                }
            });

            // 发送新的配置参数到 ESP32
            client.publish(configTopic, sentConfig, (err) => {
                if (!err) {
                    console.log('New configuration sent:', sentConfig);
                } else {
                    console.log('Failed to send configuration:', err);
                }
            });
        });
    } else {
        // 保持原有逻辑，不发送配置，只订阅device/ele_run主题
        client.on('connect', () => {
            console.log(`Connected to MQTT server with Client ID: ${clientId}`);

            // 订阅特定的主题
            client.subscribe(topic, (err) => {
                if (!err) {
                    console.log(`Subscribed to topic: ${topic}`);
                } else {
                    console.log(`Failed to subscribe: ${err}`);
                }
            });
        });

        console.log('保持现有逻辑，不发送新配置。');
    }

    rl.close();
});

// 监听收到的消息
client.on('message', (topic, message) => {
    console.log(`Client ID: ${clientId}, Received message on topic ${topic}: ${message.toString()}`);

  // 如果收到的是 ESP32 返回的配置信息
  if (topic === configResponseTopic) {
    console.log(`Received config response from ESP32: ${message.toString()}`);
    
    // 比较发送的参数和接收到的参数
    if (message.toString() === sentConfig) {
        console.log('Configuration matches!');

        // 添加二次确认逻辑
        const rlConfirm = readline.createInterface({
            input: process.stdin,
            output: process.stdout
        });

        // 提示用户确认是否发送 "OK"
        rlConfirm.question('配置匹配，是否发送确认消息 OK？(1: 确认, 0: 取消)：', (confirmInput) => {
            if (confirmInput === '1') {
                // 如果用户确认发送 OK
                client.publish(confirmationTopic, 'OK', (err) => {
                    if (!err) {
                        console.log('OK sent, configuration confirmed.');
                    } else {
                        console.log('Failed to send OK:', err);
                    }
                });
            } else {
                console.log('用户取消了发送 OK。');
            }
            rlConfirm.close();
        });
    } else {
        console.log('Configuration does not match, retrying...');
    }
}
});

// 处理断开连接
client.on('close', () => {
    console.log('Disconnected from MQTT server');
});