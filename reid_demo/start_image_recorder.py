from aideck import Recorder
import time

def main():
    connector = Recorder()

    while connector.running:
        time.sleep(1)



if __name__ == "__main__":
    main()