package com.example.spotter.View;

import static com.example.spotter.Controller.NotificationHelper.BACK;
import static com.example.spotter.Controller.NotificationHelper.KNEE;


import android.Manifest;
import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;
import androidx.fragment.app.FragmentManager;


import android.os.CountDownTimer;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;


import com.example.spotter.Controller.NotificationHelper;
import com.example.spotter.Model.FlexSensor;
import com.example.spotter.Model.ImuSensor;
import com.google.firebase.auth.FirebaseAuth;
import com.example.spotter.R;
import com.example.spotter.Controller.DataBaseHelper;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.util.Map;

public class SquatActivity extends AppCompatActivity {

    private boolean timerPaused = false;

    private boolean timerRunning = false;

    private CountDownTimer countDownTimer;

    private int counter;

    static final String Lobster = "Lobster_Squat";

    private TextView flex_text, relativeAngleX_text, flexWarning, imuWarning;
    private TextView clockTextView;
    private Button chartButton, startClockButton, resetClockButton, stopAcqBtn;
    private DataBaseHelper db;
    private NotificationManagerCompat notificationManager;
    Context context = this;
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;
    boolean stopAcq;

    DatabaseReference sensorDatabase = FirebaseDatabase.getInstance().getReference("Sensor"); // choose the correct pathing
    DatabaseReference flagDatabase = FirebaseDatabase.getInstance().getReference("Flags"); //path for flag

    private View.OnClickListener helpActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to Help");
            FragmentManager fm = getSupportFragmentManager();
            HelpFragmentSquats hp = new HelpFragmentSquats();
            hp.show(fm, "fragment_help_squat");
        }
    };


    @Override
    protected void onCreate(Bundle savedInstanceState) {

        // Inflate the layout for this fragment
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_squats);
        db = new DataBaseHelper(SquatActivity.this);
        flex_text = findViewById(R.id.squat_flex);
        relativeAngleX_text = findViewById(R.id.squat_ra1);
        chartButton = findViewById(R.id.chartButton);
        stopAcqBtn = findViewById(R.id.stopAcqBtn);
        chartButton.setVisibility(View.INVISIBLE);

        //Warnings
        imuWarning = findViewById(R.id.imuWarning);
        flexWarning = findViewById(R.id.flexWarning);

        //Notification
        notificationManager =  NotificationManagerCompat.from(this);

        sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE);
        editor = sharedPreferences.edit();

        //Delete Database if needed
        //db.DeleteDatabase(this);


        //startClockButton = findViewById(R.id.startClockButton);
        //resetClockButton = findViewById(R.id.resetClockButton);
        //clockTextView = findViewById(R.id.clockTextView);

        /*startClockButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (timerRunning) {
                    if (timerPaused) {
                        resumeTimer();
                    } else {
                        pauseTimer();
                    }
                } else {
                    startTimer();
                }
            }
        });*/

        /*resetClockButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                resetTimer();
            }
        });*/

        getSupportActionBar().setTitle("Squats");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        chartButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d(Lobster, "Go to charts");
                int id = sharedPreferences.getInt("id", -1);
                if(id != -1) {
                    editor.putInt("id_chart", id);
                } else {
                    Log.w(Lobster, "Failed to get id for chart view.");
                }
                Intent intent = new Intent(SquatActivity.this, LineChartView.class);
                startActivity(intent);
            }
        });

        stopAcqBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                stopAcqn();
                stopAcq = true;
                stopAcqBtn.setVisibility(View.INVISIBLE);
            }
        });

    }

    @Override
    protected void onResume() {
        super.onResume();
        //UpdateRealTimeData();

    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onPostResume() {
        super.onPostResume();
        stopAcq = false;
        UpdateRealTimeData();
    }

    /*private void startTimer() {
        timerRunning = true;
        countDownTimer = new CountDownTimer(30000, 1000) {
            public void onTick(long millisUntilFinished) {
                counter++;
                clockTextView.setText(String.valueOf(counter));
            }

            public void onFinish() {
                clockTextView.setText("Stop");
                stopTimer();
            }
        }.start();
    }

    private void pauseTimer() {
        if (countDownTimer != null) {
            countDownTimer.cancel();
        }
        timerPaused = true;
    }

    private void resumeTimer() {
        timerRunning = true;
        timerPaused = false;
        countDownTimer = new CountDownTimer((30 - counter) * 1000, 1000) {
            public void onTick(long millisUntilFinished) {
                counter++;
                clockTextView.setText(String.valueOf(counter));
            }

            public void onFinish() {
                clockTextView.setText("Stop");
                stopTimer();
            }
        }.start();
    }

    private void stopTimer() {
        if (countDownTimer != null) {
            countDownTimer.cancel();
        }
        timerRunning = false;
        timerPaused = false;
        counter = 0;
        clockTextView.setText(String.valueOf(counter));
    }

    private void resetTimer() {
        if (countDownTimer != null) {
            countDownTimer.cancel();
        }
        timerRunning = false;
        timerPaused = false;
        counter = 0;
        clockTextView.setText(String.valueOf(counter));
    }*/


    private void UpdateRealTimeData() {

        sensorDatabase.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {

                Map<String, Double> value = (Map<String, Double>) snapshot.getValue(true);
                FlexSensor flex = new FlexSensor(value.get("Flex"));
                ImuSensor imu = new ImuSensor(value.get("Angle1x"), value.get("Angle1y"), value.get("Angle2x"), value.get("Angle2y"));

                squatBackNotification(flex);
                squatKneeNotification(imu);

                flex_text.setText(String.valueOf(flex.getFlex()));
                relativeAngleX_text.setText(String.valueOf(imu.getRelative_x()));
                updateWarning(imu.getRelative_x(),flex.getFlex());

                boolean isDatabaseEmpty = db.isDatabaseEmpty();
                if (isDatabaseEmpty) {
                    // Database is empty
                    editor.putInt("id", 1); // Replace "key" with your context identifier and "value" with the actual data to be passed.
                    editor.apply();
                    db.insertSensors(flex, imu, "Squats", 1);
                } else {
                    //Database is not empty
                    int id = sharedPreferences.getInt("id", -1);
                    if (id != -1) {
                        db.insertSensors(flex, imu, "Squats", id);
                    }
                    else {
                        Toast.makeText(context, "Storing sensor data into database failed", Toast.LENGTH_LONG).show();
                    }
                }


                //Log.d(Lobster, "Value is: " + value);

            }
            @Override
            public void onCancelled(@NonNull DatabaseError error) {
                Log.w(Lobster, "Failed to retrieve value.");
            }
        });
    }
    public void squatBackNotification(FlexSensor f) { // have to have notifications enabled

        if (f.getFlex() >= 5 ) { // below 5 Excellent; between 5 and 15 back is slightly bent
            if(f.getFlex() > 15 ){ // above 15 means back is over bent

                Notification notification = new NotificationCompat.Builder(this , BACK)
                        .setSmallIcon(R.drawable.error_notification)
                        .setContentTitle("Squat Error")
                        .setContentText("Back is over bending! Fix form.")
                        .setPriority(NotificationCompat.PRIORITY_HIGH)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(0, notification);
            }else {
                Notification notification = new NotificationCompat.Builder(this, BACK)
                        .setSmallIcon(R.drawable.warning_notification)
                        .setContentTitle("Squat Error")
                        .setContentText("Back is starting to bend.")
                        .setPriority(NotificationCompat.PRIORITY_DEFAULT)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(0, notification);
                //Log.e(Lobster, "Notification Created");
            }
        }else{
            //do nothing
        }
    }
    public void squatKneeNotification(ImuSensor i) { // have to have notifications enabled

        if (i.getRelative_x() >= 100) { // Between 80 and 100 Good; between 100 and 110 caution on depth
            if (i.getRelative_x() > 110) { // above 110 is a warning for too deep

                Notification notification = new NotificationCompat.Builder(this, KNEE)
                        .setSmallIcon(R.drawable.error_notification)
                        .setContentTitle("Squat Warning")
                        .setContentText("Squat is much too deep")
                        .setPriority(NotificationCompat.PRIORITY_HIGH)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(1, notification);
            } else {
                Notification notification = new NotificationCompat.Builder(this, KNEE)
                        .setSmallIcon(R.drawable.warning_notification)
                        .setContentTitle("Squat Caution")
                        .setContentText("Squat is deep.")
                        .setPriority(NotificationCompat.PRIORITY_DEFAULT)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(1, notification);
                //Log.e(Lobster, "Notification Created");
            }
        } else {
            //do nothing
        }
    }

    private void stopAcqn(){
        if (!stopAcq) {
            flagDatabase.child("stopRead").setValue(true);
            chartButton.setVisibility(View.VISIBLE);
            int id = sharedPreferences.getInt("id", -1);
            if(id != -1) {
                editor.putInt("id_chart", id);
                id++; //increment id for next acq
                editor.putInt("id", id);
                editor.apply();
            } else {
                Log.w(Lobster, "Failed to increment id value.");
            }
        }
    }

    private void updateWarning (double imu, double flex){
        if (imu > 110) {
            imuWarning.setText("Warning: Squat is much too deep");
        }
        else if (imu > 100) {
            imuWarning.setText("Caution: Squat is deep");
        }
        else if (imu > 80) {
            imuWarning.setText("Great Squat");
        }
        else if (imu > 70){
            imuWarning.setText("Try going into a deeper squat");
        }
        else {
            imuWarning.setText("");
        }
        if (flex > 15){
            flexWarning.setText("Back is bent. Adjust form");
        } else if(flex > 5) {
            flexWarning.setText("Back is slightly bent");
        }
        else {
            flexWarning.setText("Excellent");
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.clock_bar, menu);
        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (item.getItemId() == android.R.id.home) {
            // Home button clicked
            if (!stopAcq){
                stopAcqn();
            }
            Intent intent = new Intent(this, HomeActivity.class);
            startActivity(intent);
            return true;
        }
        else if (id == R.id.menu_clock_activity) {
            Log.d(Lobster, "Go to Clock Dialog Fragment");
            FragmentManager fm = getSupportFragmentManager();
            ClockFragment hp = new ClockFragment();
            hp.show(fm, "clock_fragment");
            return true;
        }
        return(super.onOptionsItemSelected(item));
    }

}