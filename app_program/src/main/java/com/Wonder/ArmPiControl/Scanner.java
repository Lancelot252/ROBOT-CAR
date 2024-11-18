package com.Wonder.ArmPiControl;

/**
 * Created by andy on 2017/9/18.
 */

import android.content.Context;//上下文对象  用于获取系统服务
import android.net.DhcpInfo;//DHCP信息  用于获取DHCP信息
import android.net.wifi.WifiManager;//wifi管理器    用于获取wifi信息和控制wifi状态
import android.net.wifi.WifiManager.MulticastLock;//多播锁  用于多播数据包的发送和接收
import android.os.AsyncTask;//异步任务  用于在后台执行耗时操作

import java.io.IOException;//输入输出异常
import java.math.BigInteger;//大整数类  用于处理超过64位的整数
import java.net.DatagramPacket;//数据报包  用于发送和接收数据报包
import java.net.DatagramSocket;//数据报套接字  用于发送和接收数据报包
import java.net.InetAddress;//IP地址类  用于表示IP地址
import java.net.InetSocketAddress;//套接字地址  用于表示IP地址和端口号
import java.net.SocketException;//套接字异常  用于处理套接字相关的异常
import java.net.UnknownHostException;//未知主机异常  用于处理无法确定主机IP地址的异常
import java.util.HashMap;//哈希映射  用于存储键值对
import java.util.Map;//映射接口  用于定义键值对存储结构

public class Scanner {//定义Scanner类
    private Context _context;//上下文对象  用于获取系统服务
    private Scanner.OnScanOverListener _onScanOverListener;//扫描结束监听器  用于回调扫描结束事件
    private boolean _getFirst;//是否获取第一个结果的标志
    private boolean _scanning = false;//是否正在扫描的标志
    private WifiManager _wifiManager;//wifi管理器  用于获取wifi信息和控制wifi状态
    private MulticastLock _multicastLock;//多播锁  用于多播数据包的发送和接收

    public Scanner(Context context) {//构造函数  初始化Scanner对象
        this._context = context;//上下文对象  用于获取系统服务
        this._wifiManager = (WifiManager) this._context.getSystemService(Context.WIFI_SERVICE);//获取wifi服务  用于获取wifi信息和控制wifi状态
    }

    private boolean scan(boolean getFirst) {//扫描方法  开始扫描
        if (this._scanning) {//如果正在扫描
            return false;//返回false  表示扫描未开始
        } else {//否则
            this._scanning = true;//设置正在扫描标志为true
            this._getFirst = getFirst;//设置是否获取第一个结果的标志
            this._multicastLock = this._wifiManager.createMulticastLock("UDPwifi");//创建多播锁
            this._multicastLock.acquire();//获取多播锁
            Scanner.ScanAsyncTask scanAsyncTask = new Scanner.ScanAsyncTask();//创建异步任务
            scanAsyncTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, new Void[0]);//在线程池中执行异步任务
            return true;//返回true  表示扫描已开始
        }//线程池执行异步任务
    }

    private InetAddress getGatewayAddress() {//获取网关地址方法
        DhcpInfo dhcp = this._wifiManager.getDhcpInfo();//获取DHCP信息

        byte[] ip = BigInteger.valueOf((long) dhcp.serverAddress).toByteArray();//将服务器地址转换为字节数组
        InetAddress gatewayAddress = null;//初始化网关地址

        try {
            gatewayAddress = InetAddress.getByAddress(ip);//通过字节数组获取网关地址
        } catch (UnknownHostException var5) {//捕获未知主机异常

        }

        return gatewayAddress;//返回网关地址
    }


    private byte[] copyOfRange(byte[] from, int start) {//复制字节数组范围方法
        int index = 0;//初始化索引

        for (int i = 0; i < from.length; i++) {//遍历字节数组
            if (from[i] == 0x3A) {//如果字节为冒号
                index = i;//设置索引为当前索引
            }
        }

        int length = index + 9;//设备id固定12个字符

        byte[] result = new byte[length];//初始化结果字节数组

        System.arraycopy(from, start, result, 0, length);//复制字节数组范围
        return result;//返回结果字节数组
    }

    public boolean scanAll() {//扫描所有方法
        return this.scan(false);//调用扫描方法  参数为false
    }

    public boolean scan() {//扫描方法
        return this.scan(true);//调用扫描方法  参数为true
    }

    public void setOnScanOverListener(Scanner.OnScanOverListener listener) {//设置扫描结束监听器方法
        this._onScanOverListener = listener;//设置扫描结束监听器
    }

    public interface OnScanOverListener {//扫描结束监听器接口
        void onResult(Map<InetAddress, String> var1, InetAddress var2);//结果回调方法
    }

    private class ScanAsyncTask extends AsyncTask<Void, Void, Map<InetAddress, String>> {//异步任务类
        private int deviceIdBeginIndex;//设备id起始索引

        private ScanAsyncTask() {//构造函数  初始化异步任务
            this.deviceIdBeginIndex = 0;//初始化设备id起始索引
        }

        protected Map<InetAddress, String> doInBackground(Void... voids) {//后台执行方法
            Map<InetAddress, String> list = new HashMap();//初始化结果列表

            for (int j = 0; j < 10; ++j) {//循环10次
                DatagramSocket socket = null;//初始化数据报套接字
                try {
                    if (socket == null) {//如果数据报套接字为空
                        socket = new DatagramSocket(null);//创建数据报套接字
                        socket.setReuseAddress(true);//设置重用地址
                        socket.bind(new InetSocketAddress(9025));//绑定套接字地址
                        socket.setSoTimeout(350);//设置超时时间
                    }
                    String netAddress = "255.255.255.255";//广播地址
                    String sendStr = "LOBOT_NET_DISCOVER";//发送字符串
                    byte[] bufSend = sendStr.getBytes();//将发送字符串转换为字节数组
                    InetAddress ipaddress = InetAddress.getByName(netAddress);//获取广播地址
                    DatagramPacket datagramPacket = new DatagramPacket(bufSend, bufSend.length, ipaddress, 9027);//创建数据报包

                    // 发送数据
                    socket.send(datagramPacket);//发送数据报包
                    try {
                        Thread.sleep(100);//线程休眠100毫秒
                    } catch (Exception e) {//捕获异常
                        e.printStackTrace();//打印异常信息
                    }

                    for (int i = 0; i < 10; ++i) {//循环10次
                        try {
                            byte[] buf = new byte[30];//初始化接收字节数组
                            DatagramPacket receivedPacket = new DatagramPacket(buf, buf.length);//创建接收数据报包

                            socket.receive(receivedPacket);//接收数据报包

                            InetAddress address = InetAddress.getByName(receivedPacket.getAddress().getHostAddress());//获取发送方地址
                            if (!list.containsKey(address) && receivedPacket.getLength() > this.deviceIdBeginIndex) {//如果结果列表不包含该地址且接收数据长度大于设备id起始索引
                                String deviceId = new String(Scanner.this.copyOfRange(buf, this.deviceIdBeginIndex));//获取设备id
                                list.put(address, deviceId);//将地址和设备id添加到结果列表
                                if (Scanner.this._getFirst) {//如果获取第一个结果的标志为true
                                    return list;//返回结果列表
                                }
                            }
                        } catch (SocketException var18) {//捕获套接字异常
                            var18.printStackTrace();//打印异常信息
                        }
                    }
                } catch (SocketException var19) {//捕获套接字异常
                    var19.printStackTrace();//打印异常信息
                } catch (IOException var20) {//捕获输入输出异常
                    var20.printStackTrace();//打印异常信息
                } finally {
                    if (socket != null) {//如果数据报套接字不为空
                        socket.disconnect();//断开数据报套接字
                    }
                }
            }
            return list;//返回结果列表
        }

        protected void onPostExecute(Map<InetAddress, String> addresses) {//执行后方法
            super.onPostExecute(addresses);//调用父类方法
            Scanner.this._multicastLock.release();//释放多播锁
            if (Scanner.this._onScanOverListener != null) {//如果扫描结束监听器不为空
                Scanner.this._onScanOverListener.onResult(addresses, Scanner.this.getGatewayAddress());//回调结果方法

            }
            Scanner.this._scanning = false;//设置正在扫描标志为false
            System.gc();//调用垃圾回收
        }
    }
}

/*
* 肉：
* 牛后腿肉（一斤）、培根（30片）、热狗肠（15个）
* 鸡脆骨（半斤）、鸡胗（半斤）、鸡翅（10个）
* 鸡腿（10个）、鸡爪（12个）、火腿肠（一包）
*
* 海鲜：
* 虾（一斤）、鱿鱼（一斤）、生蚝（25个）
* 扇贝（25个）、花甲（一斤）、秋刀鱼（三条）
* 干鱿鱼（两条）、蛏子（一斤）、鲍鱼（10个）
*小黄鱼串、鲳鱼串？
*
*
* 蔬菜：生菜、玉米、金针菇、韭菜、茄子、娃娃菜、香菇、土豆、藕片、黄瓜
* 豆制品：千张、千页豆腐
* 丸子：牛丸
*
* 小馒头
*
*
*
* */
