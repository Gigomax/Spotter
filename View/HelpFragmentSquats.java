package com.example.spotter.View;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.fragment.app.DialogFragment;
import com.example.spotter.R;

public class HelpFragmentSquats extends DialogFragment {
    private TextView instructionText, textView, ContactText, figure1, figure2;

    private ImageView kneeImageView, backImageView;

    private Button returnButton;

    View.OnClickListener returnActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            getDialog().dismiss();
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View view = inflater.inflate(R.layout.fragment_help_squat, container, false);
        instructionText = view.findViewById(R.id.instructionText);
        textView = view.findViewById(R.id.textView);
        ContactText = view.findViewById(R.id.ContactText);
        figure1 = view.findViewById(R.id.figure1);
        figure2 = view.findViewById(R.id.figure2);
        kneeImageView = view.findViewById(R.id.kneeImageView);
        backImageView = view.findViewById(R.id.backImageView);


        kneeImageView.setImageResource(R.drawable.sensorknee);
        backImageView.setImageResource(R.drawable.sensorback);

        view.findViewById(R.id.returnButton).setOnClickListener(returnActivity);

        return view;
    }
}