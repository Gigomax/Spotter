package com.example.spotter.Controller;

import android.app.Application;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.content.Context;
import android.os.Build;
import android.util.Log;

public class NotificationHelper extends Application{
    static final String Lobster = "Lobster_NoteHelper";
    public static final String KNEE = "knee";
    public static final String BACK = "back";

    @Override
    public void onCreate(){
        super.onCreate();

        createNotificationChannels();

    }

    private void createNotificationChannels(){ // Create the channels for the notifications

        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.O){ // Check to make sure that you are using valid SDK version

            //Channels
            NotificationChannel kneeNote = new NotificationChannel(KNEE, "Knee", NotificationManager.IMPORTANCE_DEFAULT);
            kneeNote.setDescription("This is for setting the notifications of the squat page.");
            kneeNote.setShowBadge(true);
            NotificationChannel backNote = new NotificationChannel(BACK, "Back", NotificationManager.IMPORTANCE_DEFAULT);
            backNote.setDescription("This is for setting the notifications of the deadlift page.");

            //Setup Channels
            NotificationManager manager =  getSystemService(NotificationManager.class);
            manager.createNotificationChannel(kneeNote);manager.createNotificationChannel(backNote);

            Log.d(Lobster, "Channels have been created");
        }
    }

}
