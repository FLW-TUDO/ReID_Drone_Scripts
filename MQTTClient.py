from threading import Thread
import paho.mqtt.client as mqtt
import json

class MQTTClient(Thread):
    def __init__(self, ip, port, topics = [], publish_topic="image_detection"):
        Thread.__init__(self)
        self.client = mqtt.Client("Demo_runner")
        self.client.connect(ip, port)
        self.client.on_message = self.on_message

        self.topic_infos = {}
        self.publish_topic = publish_topic

        self.topics = topics
        for topic in self.topics:
            self.subscribe(topic)

        self.start()

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.topic_infos[msg.topic] = data
                
    def subscribe(self, topic):
        self.client.subscribe(topic, qos=2)

    def publish(self, message):
        print("Published", message, "on", self.publish_topic)
        self.client.publish(self.publish_topic, json.dumps(message), qos=2)

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