from threading import Thread
import paho.mqtt.client as mqtt
import json

class MQTTClient(Thread):
    def __init__(self, ip, port, name, topics = []):
        Thread.__init__(self)
        self.client = mqtt.Client(name)
        self.topics = topics
        self.client.on_message = self.on_message
        self.client.on_connect = self.subscribe
        self.client.connect(ip, port)

        self.topic_infos = {}

        self.start()

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.topic_infos[msg.topic] = data
                
    def subscribe(self, a, b, c, d):
        for topic in self.topics:
            self.client.subscribe(topic, qos=2)

    def publish(self, topic, message, qos=2):
        print("Published", message, "on", topic)
        self.client.publish(topic, json.dumps(message),qos=qos)

    def run(self):
        self.client.loop_forever()

    def get_bb(self, topic):
        if topic not in self.topic_infos:
            return None
        res = self.topic_infos[topic]
        self.topic_infos[topic] = None
        return res

    def close(self):
        self.client.disconnect()