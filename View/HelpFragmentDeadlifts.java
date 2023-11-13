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

public class HelpFragmentDeadlifts extends DialogFragment {
    private TextView instructionText, textView;

    private ImageView deadliftImage;

    private Button returnButton;

    View.OnClickListener returnActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            getDialog().dismiss();
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View view = inflater.inflate(R.layout.fragment_help_deadlift, container, false);
        instructionText = view.findViewById(R.id.instructionText);
        textView = view.findViewById(R.id.textView);
        deadliftImage = view.findViewById(R.id.deadliftImage);

        deadliftImage.setImageResource(R.drawable.deadlift);    // display imageView


        view.findViewById(R.id.returnButton).setOnClickListener(returnActivity);

        return view;
    }
}