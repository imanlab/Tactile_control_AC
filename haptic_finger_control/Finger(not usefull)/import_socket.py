import socket

def start_client():
    # Define the server address (host, port)
    server_address = ('169.254.172.165', 8000)

    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"connection to server...")
    # Connect to the server
    client_socket.connect(server_address)
    print(f"Connected to {server_address}")

    # Send a message to the server
    message = "Hello, server! This is the client."
    client_socket.sendall(message.encode('utf-8'))

    # Receive and print the server's response
    data = client_socket.recv(1024)
    print(f"Server response: {data.decode('utf-8')}")

    # Close the connection
    client_socket.close()

if __name__ == "__main__":
    start_client()
