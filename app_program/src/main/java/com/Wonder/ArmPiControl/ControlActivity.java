package com.Wonder.ArmPiControl;

import android.annotation.SuppressLint;
import android.app.Activity;

import android.content.Intent;

import android.net.Uri;
import android.os.AsyncTask;
import android.os.Handler;
import android.os.Message;
import android.os.Bundle;

import android.util.DisplayMetrics;

import android.util.Log;
import android.view.MotionEvent;
import android.view.View;


import android.webkit.WebView;
import android.widget.Button;
import android.widget.SeekBar;

import com.deng.netlibrary.netLib.ZyNetHttps;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;



public class ControlActivity extends Activity {
    private String _deviceIp = "";
    private WebView mVideoView;
    private ArrayList<SeekBar> seekBars = new ArrayList<>();
    private ArrayList<Button> buttons = new ArrayList<>();//前后左右移动按钮
    private ArrayList<Button> turnButtons = new ArrayList<>();//左转、右转按钮
    Timer timer = new Timer();
    @SuppressLint("ClickableViewAccessibility")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_control);

        mVideoView = (WebView) findViewById(R.id.video);
        DisplayMetrics mDisplayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(mDisplayMetrics);
        int screenWidth = mDisplayMetrics.widthPixels;
        int screenHigh = mDisplayMetrics.heightPixels;
        double ratioH = screenHigh * 100.0 / 480.0;
        double ratioW = screenWidth * 100.0 * 3 / (640.0 * 5);
        double ratio = Math.min(ratioH, ratioW);
        mVideoView.setInitialScale((int) ratio);

        Intent intent = getIntent();
        _deviceIp = intent.getStringExtra("deviceip");

        seekBars.add(findViewById(R.id.seek1));
        seekBars.add(findViewById(R.id.seek2));
        seekBars.add(findViewById(R.id.seek3));
        seekBars.add(findViewById(R.id.seek4));
        seekBars.add(findViewById(R.id.seek5));

        for (SeekBar seekBar : seekBars)
        {
            seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override
                public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                    int[] cmd = {100, 1, Integer.parseInt(seekBar.getTag().toString()), progress - 90};
                    postRpc("SetPWMServo", 1, cmd, null, null);
                }


                @Override
                public void onStartTrackingTouch(SeekBar seekBar) {

                }

                @Override
                public void onStopTrackingTouch(SeekBar seekBar) {

                }
            });
        }

        buttons.add(findViewById(R.id.forward));
        buttons.add(findViewById(R.id.backward));
        buttons.add(findViewById(R.id.left));
        buttons.add(findViewById(R.id.right));
        for (Button button : buttons)
        {
            button.setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    int tag = Integer.parseInt((String)v.getTag());
                    int[] data = new int[1];
                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN://按下  0:前进 1:后退 2:左移 3:右移
                            switch (tag)
                            {
                                case 0://Forward
                                    data[0] = 90;
                                    break;
                                case 1://Backward
                                    data[0] = 270;
                                    break;
                                case 2://Move left
                                    data[0] = 180;
                                    break;
                                case 3://Move right
                                    data[0] = 360;
                                    break;
                            }
                            postRpc("SetMovementAngle", 1, data, null, null);
                            break;
                        case MotionEvent.ACTION_MOVE://移动
                            break;
                        case MotionEvent.ACTION_UP:
                        case MotionEvent.ACTION_CANCEL:
                            data[0] = -1;
                            postRpc("SetMovementAngle", 1, data, null, null);
                            break;
                    }
                    return false;
                }
            });
        }

        turnButtons.add(findViewById(R.id.turn_left));
        turnButtons.add(findViewById(R.id.turn_right));
        for (Button button : turnButtons)
        {
            button.setOnTouchListener(new View.OnTouchListener() {
                @SuppressLint("ClickableViewAccessibility")
                @Override
                public boolean onTouch(View v, MotionEvent event) {
                    int tag = Integer.parseInt((String)v.getTag());
                    int motor1Speed = 0;// motor1 speed
                    int motor2Speed = 0;// motor2 speed
                    int motor3Speed = 0;// motor3 speed
                    int motor4Speed = 0;// motor4 speed

                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            switch (tag)
                            {
                                case 0://Turn left
                                    motor1Speed = -100;
                                    motor2Speed = 100;
                                    motor3Speed = -100;
                                    motor4Speed = 100;
                                    break;
                                case 1://Turn right
                                    motor1Speed = 100;
                                    motor2Speed = -100;
                                    motor3Speed = 100;
                                    motor4Speed = -100;
                                    break;
                            }
                            int[] data = {1, motor1Speed, 2, motor2Speed, 3, motor3Speed, 4, motor4Speed};
                            postRpc("SetBrushMotor", 1, data, null, null);
                            break;
                        case MotionEvent.ACTION_MOVE:
                            break;
                        case MotionEvent.ACTION_UP:
                        case MotionEvent.ACTION_CANCEL:
                            motor1Speed = 0;
                            motor2Speed = 0;
                            motor3Speed = 0;
                            motor4Speed = 0;
                            int[] data2 = {1, motor1Speed, 2, motor2Speed, 3, motor3Speed, 4, motor4Speed};
                            postRpc("SetBrushMotor", 1, data2, null, null);
                            break;
                    }
                    return false;
                }
            });
        }



//        timer.schedule(new TimerTask() {
//            @Override
//            public void run() {
//                postRpc("Heartbeat", 5, null, null, null);
//            }
//        }, 0, 2000);
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        postRpc("Heartbeat", 5, null, null, null);
                    }
                });
            }
        }, 0, 2000);
    }

    @Override
    protected void onResume() {
//        if (mVideoView != null)
//        {
//            String url = "http://";
//            url += _deviceIp;
//            url += ":8080/?action=stream?dummy=param.mjpg";
//            mVideoView.loadUrl(url);
//
//            int[] i = {1};//初始化开启玩法
//            postRpc("LoadFunc", 1, i, null, null);
//
//            int[] i4 = {1000, 5, 1, 0, 3, 0, 4, 0, 5, 0, 6, 0};
//            postRpc("SetPWMServo", 1, i4, null, null);
//        }

        String url = "http://";
        url += _deviceIp;
        url += ":8080/?action=stream?dummy=param.mjpg";
        mVideoView.loadUrl(url);

        int[] i = {1};//初始化开启玩法
        postRpc("LoadFunc", 1, i, null, null);

//        int[] i4 = {1000, 5, 1, 0, 3, 0, 4, 0, 5, 0, 6, 0};
        int[] i4 = {1000, 5, 1, 0, 3, 60, 4, -45, 5, 30, 6, 0};
        postRpc("SetPWMServo", 1, i4, null, null);

        super.onResume();
    }

    @Override
    protected void onDestroy() {
        postRpc("UnloadFunc", 5, null, null, null);
        mVideoView.loadDataWithBaseURL(null, "", "text/html", "utf-8", null);
        mVideoView.clearCache(true);
        mVideoView.destroy();
        mVideoView = null;
        timer.cancel();
        super.onDestroy();
    }

    //Post 发送JSON-PRC指令
    private void postRpc(String method, int type, int[] i, String[] strings, float[] floats) {
//        String urll = "http://";
//        urll += _deviceIp;
//        urll += ":9030/";
//
//        ZyNetHttps.getInstances()
//                .newBuilder(urll, method)//设置请求路径和请求方法名
//                .tag(this)//当前请求标识
//                .type(type)
//                .params_array(i, strings, floats)//请求参数
//                /*   .callBack(new GsonResponseHandler<ModelMain>() {//请求回调
//                       @Override
//                       public void onFinish(int statusCode) {
//                         //  LogHelper.e(statusCode + "");
//                       }
//
//                       @Override
//                       public void onFailure(int statusCode, String error_msg) {
//                           LogHelper.e(error_msg);
//                       }
//
//                       @Override
//                       public void onSuccess(int statusCode, ModelMain response) {
//
//                       }
//                   }) */
//                .RPC();//请求类型
        try{
            String urll = "http://";
            urll += _deviceIp;
            urll += ":9030/";

            ZyNetHttps.getInstances()
                    .newBuilder(urll, method)//设置请求路径和请求方法名
                    .tag(this)//当前请求标识
                    .type(type)
                    .params_array(i, strings, floats)//请求参数
                    /*   .callBack(new GsonResponseHandler<ModelMain>() {//请求回调
                           @Override
                           public void onFinish(int statusCode) {
                             //  LogHelper.e(statusCode + "");
                           }

                           @Override
                           public void onFailure(int statusCode, String error_msg) {
                               LogHelper.e(error_msg);
                           }

                           @Override
                           public void onSuccess(int statusCode, ModelMain response) {

                           }
                       }) */
                    .RPC();//请求类型
        }
        catch (Exception e) {
            e.printStackTrace();
            // 处理异常逻辑，例如记录日志或者提示用户网络异常
        }
    }


    private Handler mHandler = new Handler();


    public void onClick(View v)
    {
        int id = v.getId();
        if (id == R.id.left_button)
        {
            MainActivity.needScan = false;
            finish();
        }
        else if (id == R.id.stop)
        {
            int[] i = {-1};
            postRpc("SetMovementAngle", 1, i, null, null);
        }
    }
}
