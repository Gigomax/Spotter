package com.example.spotter.View;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import com.example.spotter.Controller.FirebaseHelper;
import com.example.spotter.R;
import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.auth.AuthResult;
import com.google.firebase.auth.FirebaseAuth;


public class SignUpActivity extends AppCompatActivity {
    static final String Lobster = "Lobster_SignUp"; // for Log
    public EditText usernameText, passwordText, confirmPasswordText;
    public Button confirmButton, loginButton;
    FirebaseAuth mAuth; //connect to firebase
    //private FirebaseHelper fb;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sign_in);

        getSupportActionBar().setTitle("Create Account");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        usernameText = findViewById(R.id.usernameTextS);
        passwordText = findViewById(R.id.passwordTextS);
        confirmPasswordText = findViewById(R.id.confirmPasswordText);
        confirmButton = findViewById(R.id.confirmButton);
        mAuth = FirebaseAuth.getInstance(); // Initialize Firebase

        confirmButton.setOnClickListener(new View.OnClickListener() { //button to create account
            @Override
            public void onClick(View v) {
                if(verify()){

                    String email, password;
                    email = String.valueOf(usernameText.getText());
                    password = String.valueOf(passwordText.getText());

                    mAuth.createUserWithEmailAndPassword(email, password) // from Firebase documentation for creating user https://firebase.google.com/docs/auth/android/start#java
                            .addOnCompleteListener(new OnCompleteListener<AuthResult>() {
                                @Override
                                public void onComplete(@NonNull Task<AuthResult> task) {
                                    if (task.isSuccessful()) {
                                        Log.d(Lobster, "createUserWithEmail:success");
                                        //TODO go to the next activity after success through intent.
                                        goToLogin();
                                        finish();
                                    }
                                    else {

                                        Log.w(Lobster, "createUserWithEmail:failure", task.getException());
                                        Toast.makeText(SignUpActivity.this, "Authentication failed.", Toast.LENGTH_SHORT).show();

                                    }

                                }
                            });
                }
            }
        });
    }

    public boolean verify(){ // to verify that fields are filled and match

        String email, password, confirmPassword;
        boolean flag = false;
        email = String.valueOf(usernameText.getText());
        password = String.valueOf(passwordText.getText());
        confirmPassword = String.valueOf(confirmPasswordText.getText());

        //check for an email
        if(TextUtils.isEmpty(email)){

            usernameText.setError("Email required.");
            flag = true;
        }
        //check for a password
        if(TextUtils.isEmpty(password)){

            passwordText.setError("Password required.");
            flag = true;
        }
        //check for a confirmPassword and that it matches passwordText
        if(TextUtils.isEmpty(confirmPassword)){

            confirmPasswordText.setError("Must confirm password.");
            flag = true;
        }
        else if(!confirmPassword.equals(password)) {

            confirmPasswordText.setError("Password must match.");
            flag = true;
        }
        if (flag == true){

            Toast.makeText(SignUpActivity.this,"Unable to create account.",Toast.LENGTH_SHORT).show();
            Log.w(Lobster, "Could not create account.");
            return false;
        }
        else{

            Toast.makeText(SignUpActivity.this,"Account created.",Toast.LENGTH_SHORT).show();
            Log.d(Lobster, "Account: " + email + " was created.");
            return true;
        }
    }

    private void goToLogin(){

        Intent intent = new Intent(SignUpActivity.this, HomeActivity.class);
        startActivity(intent);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        if (item.getItemId() == android.R.id.home) {
            // Home button clicked
            Intent intent = new Intent(this, MainActivity.class);
            startActivity(intent);
            return true;
        }
        return super.onOptionsItemSelected(item);
    }




}