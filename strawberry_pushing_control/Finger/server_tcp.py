import socket

def start_server():
    # Define the server address (host, port)
    server_address = ('0.0.0.0', 8000)

    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to a specific address and port
    server_socket.bind(server_address)

    # Listen for incoming connections
    server_socket.listen(1)  # 1 is the maximum number of queued connections

    print(f"Server listening on {server_address}")

    while True:
        # Wait for a connection
        print("Waiting for a connection...")
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")

        try:
            # Handle the connection
            handle_connection(client_socket)

        finally:
            # Clean up the connection
            print("Closing connection")
            client_socket.close()

def handle_connection(client_socket):
    # Receive and send data
    data = client_socket.recv(1024)
    print(f"Received: {data.decode('utf-8')}")

    message = "Hello, client! Thanks for connecting."
    client_socket.sendall(message.encode('utf-8'))

if __name__ == "__main__":
    start_server()
