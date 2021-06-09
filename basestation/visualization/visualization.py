import os
import http.server
import socketserver

PORT = 8000

os.chdir(os.path.dirname(os.path.realpath(__file__)))

Handler = http.server.SimpleHTTPRequestHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()