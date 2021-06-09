import os
import http.sevrer

def main():
    baseconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.basestation_host, virtual_host=args.basestation_vhost, credentials=credentials))
    basechannel = baseconnection.channel()
    basechannel.queue_declare(queue=args.basestation_queue, durable=True)
  
    droneconnection = pika.BlockingConnection(pika.ConnectionParameters(host=args.drone_host, virtual_host=args.drone_vhost, credentials=credentials))
    dronechannel = droneconnection.channel()
    dronechannel.queue_declare(queue=args.drone_queue, durable=True)

def loop(base_channel, drone_channel):
    def update_base():
        

    def update_drone():


    basechannel.basic_consume(queue=args.basestation_queue, auto_ack=False, on_message_callback=callback)

if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.realpath(__file__)))
    http.server()
    loop()