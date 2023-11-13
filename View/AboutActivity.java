package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.example.spotter.R;

public class AboutActivity extends AppCompatActivity {

    private TextView titleText, titleMissionText, missionText, teamTitleText, teamText, contactTitleText, contactText;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.about_fragment);

        //titleText = findViewById(R.id.titleText);
        titleMissionText = findViewById(R.id.titleMissionText);
        missionText = findViewById(R.id.missionText);
        teamTitleText = findViewById(R.id.teamTitleText);
        teamText = findViewById(R.id.teamText);
        contactTitleText = findViewById(R.id.contactTitleText);
        contactText = findViewById(R.id.contactText);

        getSupportActionBar().setTitle("About Us");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == android.R.id.home) {
            // Home button clicked
            Intent intent = new Intent(this, MainActivity.class);
            startActivity(intent);
        }
        return true;
    }

}
