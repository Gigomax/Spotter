package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;

import com.example.spotter.R;

public class ProfileActivity extends AppCompatActivity {

    static final String Lobster = "Lobster_Profile";

    private TextView accHistoryText, squatHistoryText, deadliftHistoryText;
    private ListView listHistory;
    private Button squatButton, deadliftButton;

    private View.OnClickListener squatActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to Squat Chart History");
            Intent intent = new Intent(ProfileActivity.this, SquatHistory.class);
            startActivity(intent);
        }
    };

    private View.OnClickListener deadliftActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to Deadlift Chart History");
            Intent intent = new Intent(ProfileActivity.this, DeadliftHistory.class);
            startActivity(intent);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_profile);

        listHistory = findViewById(R.id.listHistory);
        squatButton = findViewById(R.id.squatButton);
        deadliftButton = findViewById(R.id.deadliftButton);

        squatButton.setOnClickListener(squatActivity);
        deadliftButton.setOnClickListener(deadliftActivity);

        getSupportActionBar().setTitle("My Profile");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }
}