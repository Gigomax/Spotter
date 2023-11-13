package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.DialogFragment;

import android.os.Bundle;
import android.os.CountDownTimer;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import com.example.spotter.R;

public class ClockFragment extends DialogFragment {

    private TextView tvClock;

    private Button btnStart, btnReset, btnReturn;

    private boolean isTimerRunning = false;

    private long timeRemaining;

    private CountDownTimer countDownTimer;

    private final long TIMER_DURATION = 60000;
    private final long TIMER_INTERVAL = 1000;

    View.OnClickListener returnActivity = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            getDialog().dismiss();
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View view = inflater.inflate(R.layout.clock_fragment, container, false);

        tvClock = view.findViewById(R.id.tvClock);
        btnStart = view.findViewById(R.id.btnStart);
        btnReset = view.findViewById(R.id.btnReset);
        btnReturn = view.findViewById(R.id.btnReturn);

        btnReturn.setOnClickListener(returnActivity);

        btnStart.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (isTimerRunning) {
                    pauseTimer();
                } else {
                    // Start or resume the timer
                    startTimer();

                }
            }
        });

        btnReset.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // Reset the timer and UI
                if (isTimerRunning) {
                    countDownTimer.cancel();
                    isTimerRunning = false;
                }
                timeRemaining = 0;  // reset timer to 0
                updateTimerUI();
            }
        });
        return view;
    }

    private void startTimer() {
        countDownTimer = new CountDownTimer(Long.MAX_VALUE, TIMER_INTERVAL) {
            @Override
            public void onTick(long millisUntilFinished) {
                timeRemaining += TIMER_INTERVAL;    // increment time
                updateTimerUI();
            }

            @Override
            public void onFinish() {

            }
        };

        countDownTimer.start();
        isTimerRunning = true;
        btnStart.setText("Pause");
    }

    private void updateTimerUI() {
        long seconds = timeRemaining / 1000;
        long minutes = seconds / 60;
        long hours = minutes / 60;

        String timeFormatted = String.format("%02d:%02d:%02d", hours, minutes % 60, seconds % 60);
        tvClock.setText(timeFormatted);
    }

    private void pauseTimer() {
        countDownTimer.cancel();
        isTimerRunning = false;
        btnStart.setText("Start");
    }
}

