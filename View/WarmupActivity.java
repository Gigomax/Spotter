package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.webkit.WebChromeClient;
import android.webkit.WebView;

import com.example.spotter.R;

public class WarmupActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_warmup);

        //Deadlift Video (youtube, less space)
        WebView webViewDeadlift = findViewById(R.id.webViewDeadlift);
        String videoD = "<iframe width=\"100%\" height=\"100%\" src=\"https://www.youtube.com/embed/g6Tye3h5DCI\" title=\"YouTube video player\" frameborder=\"0\" allow=\"accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share\" allowfullscreen></iframe>";
        webViewDeadlift.loadData(videoD,"text/html","utf-8");
        webViewDeadlift.getSettings().setJavaScriptEnabled(true);
        webViewDeadlift.setWebChromeClient(new WebChromeClient());

        //Squats Video (youtube, less space)
        WebView webViewSquats = findViewById(R.id.webViewSquats);
        String videoS = "<iframe width=\"100%\" height=\"100%\" src=\"https://www.youtube.com/embed/EG4YKX3-Ygk\" title=\"YouTube video player\" frameborder=\"0\" allow=\"accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share\" allowfullscreen></iframe>";
        webViewSquats.loadData(videoS,"text/html", "utf-8");
        webViewSquats.getSettings().setJavaScriptEnabled(true);
        webViewSquats.setWebChromeClient(new WebChromeClient());

        getSupportActionBar().setTitle("Warm ups");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }
}