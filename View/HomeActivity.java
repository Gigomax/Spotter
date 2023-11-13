package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.fragment.app.FragmentManager;


import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;

import com.example.spotter.R;
import com.google.firebase.auth.FirebaseAuth;
import com.google.firebase.auth.FirebaseUser;

public class HomeActivity extends AppCompatActivity {

    static final String Lobster = "Lobster_Home";

    private Button squatsButton, deadliftsButton, warmupButton, logOut;

    private FirebaseUser user;

    private View.OnClickListener warmupActivity = new View.OnClickListener() { //Vinnie
        @Override
        public void onClick(View v) {
            Intent intent = new Intent(getBaseContext(), WarmupActivity.class);
            startActivity(intent);
        }
    };

    private View.OnClickListener squatsActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Intent intent = new Intent(getBaseContext(), ChartMain.class);
            SharedPreferences sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE);
            SharedPreferences.Editor editor = sharedPreferences.edit();
            editor.putString("callingActivity", "squats"); // Replace "key" with your context identifier and "value" with the actual data to be passed.
            editor.apply();
            startActivity(intent);
        }
    };

    private View.OnClickListener deadliftsActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            Intent intent = new Intent(getBaseContext(), ChartMain.class);
            SharedPreferences sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE);
            SharedPreferences.Editor editor = sharedPreferences.edit();
            editor.putString("callingActivity", "deadlifts"); // Replace "key" with your context identifier and "value" with the actual data to be passed.
            editor.apply();
            startActivity(intent);
        }

        private View.OnClickListener profileActivity = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(Lobster, "Go to Profile");
                Intent intent = new Intent(getBaseContext(), ProfileActivity.class);
                startActivity(intent);
            }
        };

        private View.OnClickListener helpActivity = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(Lobster, "Go to Help");
                FragmentManager fm = getSupportFragmentManager();
                HelpFragmentSquats hp = new HelpFragmentSquats();
                hp.show(fm, "fragment_help_squat");
            }
        };
    };

//    @Override
//    public boolean onOptionsItemSelected(MenuItem item) {
//        if (item.getItemId() == android.R.id.home) {
//            // Home button clicked
//            FirebaseAuth.getInstance().signOut(); //added to sign out
//            Intent intent = new Intent(this, MainActivity.class);
//            startActivity(intent);
//            return true;
//        }
//        return super.onOptionsItemSelected(item);
//    }

    private void LogOut() {
        Intent intent = new Intent(HomeActivity.this, MainActivity.class);
        startActivity(intent);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_home);

        getSupportActionBar().setTitle("Home");
        getSupportActionBar().setDisplayHomeAsUpEnabled(false);

        squatsButton = findViewById(R.id.squatsButton);
        deadliftsButton = findViewById(R.id.deadliftsButton);
        warmupButton = findViewById(R.id.warmupButton);  //Vinnie
        //logOut = findViewById(R.id.logOutbutton);

        squatsButton.setOnClickListener(squatsActivity);
        deadliftsButton.setOnClickListener(deadliftsActivity);
        warmupButton.setOnClickListener(warmupActivity); //Vinnie

/*
        logOut.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(Lobster, "Logged out.");
                FirebaseAuth.getInstance().signOut();
                LogOut();
                finish();
            }
        });
*/
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.activity_bar, menu);
        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.menu_profile_activity) {
            Log.d(Lobster, "Go to My Profile");
            Intent intent = new Intent(getBaseContext(), ProfileActivity.class);
            startActivity(intent);
            return true;
        } else if (id == R.id.menu_settings_activity) {
            Log.d(Lobster, "Go to Settings");
            Intent intent = new Intent(getBaseContext(), SettingsActivity.class);
            startActivity(intent);
            return true;
        } else if (id == R.id.menu_logOut_activity) {
            Log.d(Lobster, "Logged out.");
            FirebaseAuth.getInstance().signOut();
            LogOut();
            finish();
            return true;
        }
        return(super.onOptionsItemSelected(item));
    }



}
