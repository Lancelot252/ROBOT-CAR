package com.Wonder.ArmPiControl;

import android.Manifest;
import android.app.Activity;

import android.content.Intent;

import android.content.pm.PackageManager;

import android.support.v4.app.ActivityCompat;
import android.os.Bundle;

import android.view.View;

import android.widget.AdapterView;

import android.widget.TextView;


import java.net.InetAddress;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class MainActivity extends TitleActivity {

    public static Scanner _scanner;
    private boolean confirm;
    HorizontalListView hListView;
    public static HorizontalListViewAdapter hListViewAdapter;
    private TextView searchingInfoText;
    public static boolean needScan = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        setLeftBtn(false);
        showLeftBtn(true);
        showRightBtn(true);
        RefreshBtnAnimStart(true);


        hListView = (HorizontalListView) findViewById(R.id.horizon_listview);
        hListViewAdapter = new HorizontalListViewAdapter(this);
        hListView.setAdapter(hListViewAdapter);
        searchingInfoText = (TextView) findViewById(R.id.searchInfoText);
        _scanner = new Scanner(this);
        _scanner.setOnScanOverListener(new Scanner.OnScanOverListener() {
            @Override
            public void onResult(Map<InetAddress, String> data, InetAddress gatewayAddress) {

                if (data != null) {//扫描到的设备
                    for (Map.Entry<InetAddress, String> entry : data.entrySet()) {
                        String id = entry.getValue();
                        String ip = entry.getKey().getHostAddress();

                        for (int i = 0; i < hListViewAdapter.getCount(); i++) {
                            HashMap<String, String> mapSearch = hListViewAdapter.getItem(i);
                            if (mapSearch.get("item_id").equals(id)) {
                                hListViewAdapter.remove(i);
                            }
                        }
                        //添加到列表
                        HashMap<String, String> map = new HashMap<String, String>();
                        map.put("item_id", id);
                        map.put("item_ip", ip);
                        hListViewAdapter.add(map);
                    }
                    searchingInfoText.setText(R.string.device_list);
                }
                if (hListViewAdapter.getCount() == 0) {
                    searchingInfoText.setText(R.string.no_find_device);
                }

                RefreshBtnAnimStart(false);
            }
        });


        hListView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                final HashMap<String, String> mapElement = hListViewAdapter.getItem(position);
                final Intent intent = new Intent();
                intent.setClass(getBaseContext(), ControlActivity.class);
                intent.putExtra("deviceid", mapElement.get("item_id"));
                intent.putExtra("deviceip", mapElement.get("item_ip"));
                startActivity(intent);
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();
        if (needScan) {
            RefreshBtnAnimStart(true);
            hListViewAdapter.clear();
            _scanner.scanAll();
            searchingInfoText.setText(R.string.searching);
        } else {
            needScan = true;
        }
    }

    @Override
    protected void onRightBtn(View forwardView) {
        RefreshBtnAnimStart(true);
        hListViewAdapter.clear();
        _scanner.scanAll();
        searchingInfoText.setText(R.string.searching);
    }

    @Override
    public void onBackPressed() {
        if (!confirm) {
            confirm = true;
            Timer timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    confirm = false;
                }
            }, 2000);
        } else {
            super.onBackPressed();
            android.os.Process.killProcess(android.os.Process.myPid());
        }
    }

    @Override
    protected void onBackward(View backwardView) {//帮助
        PackageManager manager = this.getPackageManager();
        String versionName;
        try {
            versionName = manager.getPackageInfo(this.getPackageName(), PackageManager.GET_ACTIVITIES).versionName;
        } catch (PackageManager.NameNotFoundException e) {
            e.printStackTrace();
            versionName = "1.0";
        }

        AboutUsDialog.createDialog(getFragmentManager(), versionName);
    }

}
