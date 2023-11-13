package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.app.AppCompatDelegate;
import androidx.fragment.app.FragmentManager;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;

import com.example.spotter.R;
import com.google.android.material.switchmaterial.SwitchMaterial;

import java.util.Objects;

public class SettingsActivity extends AppCompatActivity {
    static final String Lobster = "Lobster_Settings";

    private Button buttonAcc, notifButton, languageButton;
    private SwitchMaterial lightModeButton;

    private TextView ContactText;

    private static final String PREFS_NAME = "MyPrefs";
    private static final String NIGHT_MODE_KEY = "NightMode";

    private View.OnClickListener notifActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to notifications");
            Intent intent = new Intent(SettingsActivity.this, NotificationsActivity.class);
            startActivity(intent);
        }
    };

    private View.OnClickListener accActivity = new View.OnClickListener() {
    @Override
    public void onClick(View v) {
        Log.d(Lobster, "Go to Account Settings");
        Intent intent = new Intent(SettingsActivity.this, AccountActivity.class);
        startActivity(intent);
    }
};

    private View.OnClickListener languageFragment = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to languages");
            FragmentManager fm = getSupportFragmentManager();
            LanguageFragment hp = new LanguageFragment();
            hp.show(fm, "fragment_language");
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        ContactText = findViewById(R.id.ContactText);
        languageButton = findViewById(R.id.languageButton);
        notifButton = findViewById(R.id.notifButton);
        buttonAcc = findViewById(R.id.buttonAcc);

        languageButton.setOnClickListener(languageFragment);
        notifButton.setOnClickListener(notifActivity);
        buttonAcc.setOnClickListener(accActivity);



        lightModeButton = findViewById(R.id.lightModeButton);
        Objects.requireNonNull(getSupportActionBar()).setTitle("Dark-Light Mode Switch");

        // Set the checked state and text of the switch based on the current night mode
        SharedPreferences preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        boolean isNightModeOn = preferences.getBoolean(NIGHT_MODE_KEY, false);
        lightModeButton.setChecked(isNightModeOn);
        lightModeButton.setText(isNightModeOn ? "Dark Mode" : "Light Mode");

        lightModeButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES);
                    buttonView.setText("Dark Mode");
                } else {
                    AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO);
                    buttonView.setText("Light Mode");
                }
                // Save the state to SharedPreferences when the mode is changed
                SharedPreferences.Editor editor = preferences.edit();
                editor.putBoolean(NIGHT_MODE_KEY, isChecked);
                editor.apply();
            }
        });

        // Set the default night mode of the AppCompatDelegate based on the saved preference
        AppCompatDelegate.setDefaultNightMode(isNightModeOn ? AppCompatDelegate.MODE_NIGHT_YES : AppCompatDelegate.MODE_NIGHT_NO);

        getSupportActionBar().setTitle("Settings");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }

    @Override
    public void recreate() {
        finish();
        overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out);
        startActivity(getIntent());
        overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out);
    }
}
