from internal_communication import InternalCom

if __name__ == "__main__":
    orin = InternalCom(no_thread=True)
    orin.send_messages()
