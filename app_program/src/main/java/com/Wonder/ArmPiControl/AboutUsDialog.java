package com.Wonder.ArmPiControl;

import android.app.DialogFragment;
import android.app.FragmentManager;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;



/**
 * Created by andy on 2017/9/25.
 */

public class AboutUsDialog extends DialogFragment implements View.OnClickListener{

    private String versionNum;
    private TextView appVersionTx;
    private Button okBtn;
    public static void createDialog(FragmentManager fragmentManager, String version) {
        AboutUsDialog dialog = new AboutUsDialog();
        dialog.versionNum = version;
        dialog.show(fragmentManager, "searchDialog");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setStyle(STYLE_NO_TITLE, getTheme());
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.layout_about, container, false);

    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        appVersionTx = (TextView)view.findViewById(R.id.app_version);
        okBtn = (Button)view.findViewById(R.id.dialog_btn);
        okBtn.setOnClickListener(this);
        String appVersionStr = getString(R.string.app_version);
        appVersionStr += "V";
        appVersionStr += String.valueOf(versionNum);
        appVersionTx.setText(appVersionStr);
    }

    public void onDestroyView() {
        super.onDestroyView();
    }

    @Override
    public void onClick(View v) {
        dismissAllowingStateLoss();
    }
}