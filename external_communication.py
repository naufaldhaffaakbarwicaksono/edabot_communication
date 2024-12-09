import socket
import threading
import configs
from edabot_protocol import send_packet, receive_packet


class ExternalCom:
    def __init__(self):
        # Initialize Raspi as Server for Dashboard
        self.raspi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.raspi_socket.bind((configs.raspi_ip, configs.raspi_port_server))
        self.raspi_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.raspi_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.raspi_socket.listen(5)

        if configs.bypass_integrity:
            # Initialize Raspi as Server for Orin
            self.orin_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.orin_socket.bind((configs.raspi_ip_internal, configs.orin_port))
            self.orin_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.orin_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        if not configs.bypass_integrity:
            # Initialize Raspi as Client for Zinq
            self.zinq_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.zinq_socket.connect((configs.zinq_ip, configs.zinq_port_secure))

        self.dashboard_client = None
        self.orin_socket = None
        self.zinq_socket = None

        # Start threads for receiving and sending messages
        self.start()

    def start(self):
        """
        Device pada kode ini (Raspi) akan berperan sebagai server untuk Dashboard dan Orin, dan client untuk Zinq. menggunakan protokol pada library edabot_protocol.py baca dokumentasi pada file tersebut
        Membuat thread untuk menerima dan mengirim pesan dengan obyek koneksi 3 device
        1. Dasboard (komunikasi eksternal menggunakan simple_protocol=False)
        2. Orin (komunikasi internal dieksesusi jika bypass_integrity=True, komunikasi internal menggunakan simple_protocol=True)
        3. Zinq (komunikasi internal dieksesusi jika bypass_integrity=False, komunikasi internal menggunakan simple_protocol=True)
        """
        
        """
        Buat server untuk Dashboard
        Apabila bypass_integrity bernilai True, maka Raspi akan berperan sebagai server untuk Orin pada IP internal dan port internal
        Apabila bypass_integrity bernilai False, maka Raspi akan berperan sebagai client untuk Zinq pada IP Zinq dan port secure
        program ini digunakan untuk mengirim dan menerima pesan dari orin (ketika bypass_integrity=True) dan zinq (ketika bypass_integrity=False) lalu mengirimkan pesan ke dashboard
        program ini digunakan untuk menerima data dari dashboard dan mengirimkan data ke orin (ketika bypass_integrity=True) dan zinq (ketika bypass_integrity=False)
        gunakan thread untuk menerima dan mengirim pesan dengan masing-masing variabel yang berbeda agar tidak terjadi race condition
        """
        # Start thread for Dashboard server
        threading.Thread(target=self.dashboard_server).start()

        if configs.bypass_integrity:
            # Start thread for Orin server
            threading.Thread(target=self.orin_server).start()
        else:
            # Start thread for Zinq client
            threading.Thread(target=self.zinq_client).start()

    def dashboard_server(self):
        while True:
            self.dashboard_client, _ = self.raspi_socket.accept()
            threading.Thread(target=self.handle_dashboard_client, args=(self.dashboard_client,)).start()

    def handle_dashboard_client(self, client_socket):
        while True:
            try:
                data = receive_packet(client_socket, simple_protocol=False)
                if configs.bypass_integrity:
                    send_packet(self.orin_socket, data, simple_protocol=True)
                else:
                    send_packet(self.zinq_socket, data, simple_protocol=True)
            except Exception as e:
                print(f"Dashboard client error: {e}")
                client_socket.close()
                break

    def orin_server(self):
        self.orin_socket.listen(5)
        while True:
            orin_client, _ = self.orin_socket.accept()
            threading.Thread(target=self.handle_orin_client, args=(orin_client,)).start()

    def handle_orin_client(self, client_socket):
        while True:
            try:
                data = receive_packet(client_socket, simple_protocol=True)
                send_packet(self.dashboard_client, data, simple_protocol=False)
            except Exception as e:
                print(f"Orin client error: {e}")
                client_socket.close()
                break

    def zinq_client(self):
        while True:
            try:
                data = receive_packet(self.zinq_socket, simple_protocol=True)
                send_packet(self.dashboard_client, data, simple_protocol=False)
            except Exception as e:
                print(f"Zinq client error: {e}")
                self.zinq_socket.close()
                break

if __name__ == "__main__":
    external_com = ExternalCom()
