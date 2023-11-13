package com.example.spotter.View;

import androidx.fragment.app.DialogFragment;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import com.example.spotter.R;

public class InfoChartFragment extends DialogFragment {

    private Button returnBtn;
    private TextView titleText, goalText;

    View.OnClickListener returnActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            getDialog().dismiss();
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View view = inflater.inflate(R.layout.fragment_info_chart, container, false);
        titleText = view.findViewById(R.id.titleText);
        goalText = view.findViewById(R.id.goalText);

        view.findViewById(R.id.returnBtn).setOnClickListener(returnActivity);

        return view;
    }

}