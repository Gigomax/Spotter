package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.DialogFragment;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import com.example.spotter.R;

public class NotificationFragment extends DialogFragment {

    private Button returnButton;

    View.OnClickListener returnActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            getDialog().dismiss();
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_notification, container, false);

        returnButton = view.findViewById(R.id.returnButton);

        view.findViewById(R.id.returnButton).setOnClickListener(returnActivity);

        return view;
    }
}