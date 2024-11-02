const aedes = require('aedes')();
const net = require('net');

// 使用变量设置IP和端口，便于灵活配置
const MQTT_SERVER_IP = '192.168.218.1';
const MQTT_SERVER_PORT = 1883;

// 创建 MQTT 服务器
const server = net.createServer(aedes.handle);

// 监听服务器
server.listen(MQTT_SERVER_PORT, MQTT_SERVER_IP, function () {
    console.log(`MQTT 服务器正在运行，监听地址: ${MQTT_SERVER_IP}, 端口: ${MQTT_SERVER_PORT}`);
});

// 监听客户端连接事件
aedes.on('client', function (client) {
    console.log(`客户端已连接: ${client.id}`);
});

// 监听订阅事件
aedes.on('subscribe', function (subscriptions, client) {
    console.log(`客户端 ${client.id} 订阅了主题: ${subscriptions.map(s => s.topic).join(', ')}`);
});

// 监听发布事件
aedes.on('publish', function (packet, client) {
    const topic = packet.topic;
    const message = packet.payload.toString();

    // 过滤掉系统主题 $SYS 的消息
    if (topic.startsWith('$SYS')) {
        return; // 直接返回，忽略系统主题消息
    }

    // 打印收到的消息
    console.log(`收到消息: ${message} 来自客户端: ${client ? client.id : 'BROKER'}, 主题: ${topic}`);
    
    // 根据需求处理每个主题的消息，但不进行发布
    if (topic === 'device/firmware') {
        console.log('处理固件更新消息...');
    }

    if (topic === 'device/command') {
        console.log('处理命令消息...');
    }

    if (topic === 'device/debug') {
        console.log('处理调试消息...');
    }

    if (topic === 'device/ele_run') {
        console.log(`处理电压数据消息: ${message}`);
    }
});

// 监听客户端断开连接事件
aedes.on('clientDisconnect', function (client) {
    console.log(`客户端断开连接: ${client.id}`);
});
