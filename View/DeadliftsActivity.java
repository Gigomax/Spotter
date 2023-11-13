package com.example.spotter.View;

import static com.example.spotter.Controller.NotificationHelper.BACK;
import static com.example.spotter.Controller.NotificationHelper.KNEE;

import android.app.Notification;
import android.content.Context;
import android.content.Intent;
import android.media.Image;
import android.content.SharedPreferences;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;
import androidx.fragment.app.FragmentManager;


import android.os.CountDownTimer;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.spotter.Controller.DataBaseHelper;
import com.example.spotter.Model.FlexSensor;
import com.example.spotter.Model.ImuSensor;
import com.google.firebase.auth.FirebaseAuth;
import com.example.spotter.R;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.util.Map;


public class DeadliftsActivity extends AppCompatActivity {

    private boolean timerPaused = false;
    private boolean timerRunning = false;
    private CountDownTimer countDownTimer;
    private int counter;
    boolean stopAcq;
    static final String Lobster = "Lobster_Deadlift";

    private TextView flexWarning, imuWarning, flex_text, relativeAngleX_text;
    private Button helpButton, chartButton, stopAcqBtn, startClockButton, resetClockButton;
    DatabaseReference refDatabase, sensor;
    private DataBaseHelper db;
    private NotificationManagerCompat notificationManager;
    Context context = this;
    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;

    DatabaseReference sensorDatabase = FirebaseDatabase.getInstance().getReference("Sensor"); // choose the correct pathing
    DatabaseReference flagDatabase = FirebaseDatabase.getInstance().getReference("Flags"); //path for flag

    private View.OnClickListener helpActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Log.d(Lobster, "Go to Help");
            FragmentManager fm = getSupportFragmentManager();
            HelpFragmentDeadlifts hp = new HelpFragmentDeadlifts();
            hp.show(fm, "fragment_help_deadlift");
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_deadlifts);
        db = new DataBaseHelper(DeadliftsActivity.this);

        flex_text = findViewById(R.id.deadlift_flex);
        relativeAngleX_text = findViewById(R.id.deadlift_ra1);
        chartButton = findViewById(R.id.chartButton);
        stopAcqBtn = findViewById(R.id.stopAcqBtn);
        //Notification
        notificationManager =  NotificationManagerCompat.from(this);

        //Warnings
        imuWarning = findViewById(R.id.imuWarning_deadlift);
        flexWarning = findViewById(R.id.flexWarning_deadlift);

       sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE);
       editor = sharedPreferences.edit();

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
        });

        resetClockButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                resetTimer();
            }
        });*/

        getSupportActionBar().setTitle("Deadlifts");
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
                Intent intent = new Intent(DeadliftsActivity.this, LineChartView.class);
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
    protected void onPostResume() {
        super.onPostResume();
        stopAcq = false;
        UpdateRealTimeData();
    }

    private void UpdateRealTimeData(){
        refDatabase = FirebaseDatabase.getInstance().getReference("Sensor"); // choose the correct pathing

        refDatabase.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot snapshot) {

                Map<String, Double> value = (Map<String, Double>) snapshot.getValue(true);
                FlexSensor flex = new FlexSensor(value.get("Flex"));
                ImuSensor imu = new ImuSensor(value.get("Angle1x"), value.get("Angle1y"), value.get("Angle2x"), value.get("Angle2y"));

                deadliftNotification(flex);
                deadliftKneeNotification(imu);

                flex_text.setText(String.valueOf(flex.getFlex()));
                relativeAngleX_text.setText(String.valueOf(imu.getRelative_x()));
                updateWarning(imu.getRelative_x(),flex.getFlex());

                boolean isDatabaseEmpty = db.isDatabaseEmpty();
                if (isDatabaseEmpty) {
                    // Database is empty
                    editor.putInt("id", 1); // Replace "key" with your context identifier and "value" with the actual data to be passed.
                    editor.apply();
                    db.insertSensors(flex, imu, "Deadlifts", 1);
                } else {
                    //Database is not empty
                    int id = sharedPreferences.getInt("id", -1);
                    if (id != -1) {
                        db.insertSensors(flex, imu, "Deadlifts", id);
                    }
                    else {
                        Toast.makeText(context, "Database storage of sensor data failed", Toast.LENGTH_LONG).show();
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
    public void deadliftNotification(FlexSensor f) { // have to have notifications enabled

        if (f.getFlex() >= -10 ) { // below -10 Excellent; between -10 and -5 back is slightly bent
            if(f.getFlex() > -5){ // above -5 means back is over bent

                Notification notification = new NotificationCompat.Builder(this , BACK)
                        .setSmallIcon(R.drawable.error_notification)
                        .setContentTitle("Deadlift Error")
                        .setContentText("Back is over bending! Fix form.")
                        .setPriority(NotificationCompat.PRIORITY_HIGH)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(2, notification);
            }else {
                Notification notification = new NotificationCompat.Builder(this, BACK)
                        .setSmallIcon(R.drawable.warning_notification)
                        .setContentTitle("Deadlift Error")
                        .setContentText("Back is starting to bend.")
                        .setPriority(NotificationCompat.PRIORITY_DEFAULT)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(2, notification);
                //Log.e(Lobster, "Notification Created");
            }
        }else{
            //do nothing
        }
    }
    public void deadliftKneeNotification(ImuSensor i) { // have to have notifications enabled

        if (i.getRelative_x() >= 100) { // Between 80 and 100 Good; between 100 and 110 caution on depth
            if (i.getRelative_x() > 110) { // above 110 is a warning for too deep

                Notification notification = new NotificationCompat.Builder(this, KNEE)
                        .setSmallIcon(R.drawable.error_notification)
                        .setContentTitle("Deadlift Warning")
                        .setContentText("Deadlift is much too deep")
                        .setPriority(NotificationCompat.PRIORITY_HIGH)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(3, notification);
            } else {
                Notification notification = new NotificationCompat.Builder(this, KNEE)
                        .setSmallIcon(R.drawable.warning_notification)
                        .setContentTitle("Deadlift Caution")
                        .setContentText("Deadlift is deep.")
                        .setPriority(NotificationCompat.PRIORITY_DEFAULT)
                        .setOnlyAlertOnce(true)
                        .setAutoCancel(true)
                        .build();
                notificationManager.notify(3, notification);
                //Log.e(Lobster, "Notification Created");
            }
        } else {
            //do nothing
        }
    }

/*    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.clock_bar, menu);
        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.menu_clock_activity) {
            Log.d(Lobster, "Go to Clock Dialog Fragment");
            FragmentManager fm = getSupportFragmentManager();
            ClockFragment hp = new ClockFragment();
            hp.show(fm, "clock_fragment");
            return true;
        }
        return(super.onOptionsItemSelected(item));
    }*/

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
            imuWarning.setText("Warning: Deadlift is much too deep");
        }
        else if (imu > 100) {
            imuWarning.setText("Caution: Deadlift is deep");
        }
        else if (imu > 80) {
            imuWarning.setText("Great Deadlift");
        }
        else if (imu > 70){
            imuWarning.setText("Try going into a deeper Deadlift");
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
