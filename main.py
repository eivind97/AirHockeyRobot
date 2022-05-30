import os
from kivyApp import MyApp

import threading
import serial

def main():
    try:
        app = MyApp()
        app.run()
    except Exception as e:
        print("Exception: ", str(e))
        os.exit(1)


if __name__ == "__main__":
    main()