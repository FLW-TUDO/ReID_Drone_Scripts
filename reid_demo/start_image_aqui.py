from aideck import Connector
import time

def main():
    connector = Connector()

    while connector.running:
        time.sleep(1)



if __name__ == "__main__":
    main()