package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;

import com.example.spotter.R;

public class DeadliftHistory extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_deadlift_history);

        getSupportActionBar().setTitle("Deadlift Chart History");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }

}