package com.Wonder.ArmPiControl;

/**
 * Created by andy on 2017/9/22.
 */

import android.util.Log;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;


class TcpSocket {
    Socket socket;
    DataInputStream din;
    DataOutputStream dou;

    public TcpSocket() {
        socket = new Socket();
    }

    public boolean Connect(String ip, int port) {
        boolean boo = false;
        if (ip != null) {
            SocketAddress socketAddress = new InetSocketAddress(ip, port);
            Log.d("connectStatus", "socketAddress : = " + socketAddress);
            try {
                socket.connect(socketAddress, 5000);
                socket.setTcpNoDelay(true);
                socket.setKeepAlive(true);
                din = new DataInputStream(socket.getInputStream());
                dou = new DataOutputStream(socket.getOutputStream());
                boo = true;
            } catch (Exception e) {
                boo = false;
                // TODO: handle exception
            }
        }
        return boo;
    }

    public boolean setSoTimeout(int ms) {
        boolean boo = false;
        try {
            socket.setSoTimeout(ms);
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }

    public boolean Bind(int port) {
        boolean boo = false;
        try {
            socket.bind(new InetSocketAddress(port));
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }

    //��������
    public boolean Send_Byte(byte[] Data, int offset, int count) {
        boolean boo = false;
        try {
            dou.write(Data, offset, count);
            dou.flush();
            //socket.getOutputStream().write(Data);
            //socket.getOutputStream().flush();
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }

    //��������
    public boolean Send_Str(String Data) {
        boolean boo = false;
        try {
            dou.writeBytes(Data);
            dou.flush();
            //socket.getOutputStream().write(Data);
            //socket.getOutputStream().flush();
            Log.d("niubility", "发送数据 ： =" + Data);
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }

    //��������
    public boolean Send(byte[] Data) {
        boolean boo = false;
        try {
            dou.write(Data);
            dou.flush();
            //socket.getOutputStream().write(Data);
            //socket.getOutputStream().flush();
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }

    //��������
    public byte[] Read() {
        byte[] Data = null;
        try {
            int num = din.available();
            Log.d("GetPhoto1111", "num  : = " + num);
            if (num > 0) {
                Data = new byte[num];
                din.read(Data);
            }
        } catch (Exception e) {
            // TODO: handle exception
            Log.d("GetPhoto11111", "Exception  : = " + e);
        }
        return Data;
    }

    public byte[] ReadPicture() {
        byte[] Data = null;
        try {
            int available = din.available();
            if (available > 8192) {
                Data = new byte[8192];

            } else {
                Data = new byte[available];
            }
            din.read(Data);
        } catch (Exception e) {
            // TODO: handle exception
            Log.d("GetPhoto11111", "Exception  : = " + e);
        }
        return Data;
    }

    public byte[] Read_uHand() {
        byte[] Data = null;
        try {
            int available = din.available();
            if (available > 8192) {
                Data = new byte[8192];
            } else {
                Data = new byte[available];
            }
            din.read(Data);
        } catch (Exception e) {
            // TODO: handle exception
            Log.d("GetPhoto11111", "Exception  : = " + e);
        }
        return Data;
    }

    public String getip() {
        String ipString = "";
        ipString = socket.getInetAddress().getHostAddress();
        return ipString;
    }

    public int getPort() {
        int port = 0;
        port = socket.getPort();
        return port;
    }

    public boolean Close() {
        boolean boo = false;

        try {
            din.close();
            dou.close();
            socket.close();
            socket = null;
            boo = true;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return boo;
    }


}
