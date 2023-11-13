package com.example.spotter.Controller;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.auth.AuthResult;
import com.google.firebase.auth.FirebaseAuth;
import com.google.firebase.auth.FirebaseUser;
/*
*
*
*
*           This Class Currently does not work as intended.
*
*           I believe it is more of an understanding of how the Android infrastructure
*           interacts with java. I think I may need to implement it vs trying to call it.
*
* */


public class FirebaseHelper {
    private static final String Lobster = "Lobster_Firebase";
    private static FirebaseAuth mAuth;
    private Context context;

    public FirebaseHelper(Context context){

        //mAuth = FirebaseAuth.getInstance();
        this.context = context;
    }
    public static void initFirebase(){

        mAuth = FirebaseAuth.getInstance();
    }
    public boolean signIn(String email, String password) {

        final boolean[] flag = new boolean[1]; //for on success or failure

        mAuth.signInWithEmailAndPassword(email, password)
                .addOnCompleteListener((new OnCompleteListener<AuthResult>() { // from Firebase documentation for creating user https://firebase.google.com/docs/auth/android/start#java
                    @Override
                    public void onComplete(@NonNull Task<AuthResult> task) {
                        if (task.isSuccessful()) {
                            Log.d(Lobster, "signInWithEmail:success");
                            flag[0] = true;
                        } else {
                            Log.w(Lobster, "signInWithEmail:failure", task.getException());
                            Toast.makeText(context, "Login failed.", Toast.LENGTH_SHORT).show();
                            flag[0] = false;
                        }
                    }
                }));

       return flag[0];
    }
    public boolean signUp(String email, String password){

        final boolean[] flag = new boolean[1]; //for on success or failure

        mAuth.createUserWithEmailAndPassword(email, password) // from Firebase documentation for creating user https://firebase.google.com/docs/auth/android/start#java
                .addOnCompleteListener(new OnCompleteListener<AuthResult>() {
                    @Override
                    public void onComplete(@NonNull Task<AuthResult> task) {
                        if (task.isSuccessful()) {
                            Log.d(Lobster, "createUserWithEmail:success");
                            flag[0] = true;

                        }
                        else {

                            Log.w(Lobster, "createUserWithEmail:failure");//, task.getException());
                            Toast.makeText(context, "Authentication failed.", Toast.LENGTH_SHORT).show();
                            flag[0] = false;
                        }
                    }
                });
        return flag[0];
    }

    public boolean signInCheck(){

        FirebaseUser currentUser = mAuth.getCurrentUser(); // check for sign in
        if(currentUser != null){
            return true;
        }
        return false;
    }

}
