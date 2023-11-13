package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.FragmentManager;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.RadioButton;

import com.example.spotter.R;
import com.google.android.material.switchmaterial.SwitchMaterial;

import java.util.Objects;

public class NotificationsActivity extends AppCompatActivity {

    static final String Lobster = "Lobster_Notification";
    private Button soundButton;
    private SwitchMaterial popUpButton, vibrateButton;

    private View.OnClickListener notificationActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to Notification Tone");
            FragmentManager fm = getSupportFragmentManager();
            NotificationFragment hp = new NotificationFragment();
            hp.show(fm, "fragment_notification");
        }
    };


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_notifications);

        soundButton = findViewById(R.id.soundButton);
        popUpButton = findViewById(R.id.popUpButton);
        vibrateButton = findViewById(R.id.vibrateButton);

        soundButton.setOnClickListener(notificationActivity);

        Objects.requireNonNull(getSupportActionBar()).setTitle("On-Off Mode Switch");

        popUpButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    buttonView.setText("Popup Notification On\n" +
                            "(Show previews at the top of the screen)");
                } else {
                    buttonView.setText("Popup Notification Off\n" +
                            "(Show previews at the top of the screen)");
                }
            }
        });

        vibrateButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    buttonView.setText("Vibrate On");
                } else {
                    buttonView.setText("Vibrate Off");
                }
            }
        });

        getSupportActionBar().setTitle("Notifications");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }
}