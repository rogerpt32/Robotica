from socket import *

s=socket(AF_INET,SOCK_STREAM)
result=s.connect_ex(('192.168.100.1',123))
s.close()