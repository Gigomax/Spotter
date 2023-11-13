package com.example.spotter.View;

import androidx.fragment.app.DialogFragment;

import android.content.res.Configuration;
import android.content.res.Resources;
import android.os.Build;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.example.spotter.R;

import java.util.Locale;

public class LanguageFragment extends DialogFragment {

    private boolean isFrenchSelected = false;
    private RadioGroup radioGroup;
    private RadioButton radioFrench, radioEnglish;
    private TextView langText;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_language, container, false);

        radioGroup = view.findViewById(R.id.radioGroup);
        radioFrench = view.findViewById(R.id.radioFrench);
        radioEnglish = view.findViewById(R.id.radioEnglish);

        setLanguage(isFrenchSelected);

        radioGroup.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if (checkedId == R.id.radioFrench) {
                    isFrenchSelected = true;
                    setLanguage(true);
                } else if (checkedId == R.id.radioEnglish) {
                    isFrenchSelected = false;
                    setLanguage(false);
                }
            }
        });

        return view;
    }

    private void setLanguage(boolean isFrench) {
        Resources resources = getResources();
        Configuration configuration = resources.getConfiguration();
        DisplayMetrics displayMetrics = resources.getDisplayMetrics();
        if (isFrench) {
            configuration.setLocale(new Locale("fr"));
        } else {

            configuration.setLocale(Locale.ENGLISH);
        }

//        // For Android versions below Nougat (API 24), use the deprecated method
//        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.N) {
//            resources.updateConfiguration(configuration, displayMetrics);
//        } else {
//            // For Android versions Nougat (API 24) and above, use the updated method
//            // Set locale to context
//            Locale.setDefault(configuration.locale);
//            resources.updateConfiguration(configuration, displayMetrics);
//        }

        // Inform the user about the language change using a Toast
        String message = isFrench ? "Language switched to French" : "Language switched to English";
        Toast.makeText(getActivity(), message, Toast.LENGTH_SHORT).show();
    }
}