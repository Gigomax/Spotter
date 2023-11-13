package com.example.spotter.View;

import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.FragmentManager;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.Toast;

import com.example.spotter.Controller.DataBaseHelper;
import com.example.spotter.R;
import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.formatter.IndexAxisValueFormatter;
import com.github.mikephil.charting.highlight.Highlight;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;
import com.github.mikephil.charting.utils.MPPointF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class LineChartView extends AppCompatActivity {

    private LineChart lineChart;
    private List<String> xValues;
    private DataBaseHelper db;
    boolean orientationPortrait ;

    Context context;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_line_chart);

        getSupportActionBar().setTitle("Angle of Knee");
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        lineChart = findViewById(R.id.LineChart_view);

        SharedPreferences sharedPreferences = getSharedPreferences("MyPrefs", Context.MODE_PRIVATE);
        int id_acq = sharedPreferences.getInt("id_chart", -1);

        if (id_acq == -1){
            Toast.makeText(context, "Failed to retrieve Acquisition ID", Toast.LENGTH_LONG).show();
            displayDummyChart();
        }
        else {
            db = new DataBaseHelper(LineChartView.this);
            String contextData = sharedPreferences.getString("callingActivity", "default_value");
            if(contextData.equals("squats")) {
                Vector<Double> relativeXValues = db.getIMU_Relative_Angle(id_acq, "Squats");
                DisplayChart(relativeXValues);

            } else if(contextData.equals("deadlifts")) {
                Vector<Double> relativeXValues = db.getIMU_Relative_Angle(id_acq, "Deadlifts");
                DisplayChart(relativeXValues);
            } else {
                Toast.makeText(context, "Cannot retrieve exercise to display.", Toast.LENGTH_LONG).show();
                displayDummyChart();

            }
        }

        lineChart = findViewById(R.id.LineChart_view);

        lineChart.setOnChartValueSelectedListener(new OnChartValueSelectedListener() {
            @Override
            public void onValueSelected(Entry e, Highlight h) {
                float x = (float) (e.getX()*0.1);
                float y = e.getY();

                // Show a toast with the coordinates at the adjusted position
                Toast toast = Toast.makeText(LineChartView.this, "Time: " + x + ", Angle: " + y, Toast.LENGTH_SHORT);
                toast.show();

            }

            @Override
            public void onNothingSelected() {

            }
        });


        //For testing
        /*db = new DataBaseHelper(LineChartView.this);
        Vector<Double> relativeXValues = db.getIMU_Relative_Angle(9, "Squats");
        DisplayChart(relativeXValues);*/
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == android.R.id.home) {
            // Home button clicked
            Intent intent = new Intent(this, HomeActivity.class);
            startActivity(intent);
        } else if(id == R.id.menu_change) {
            if (!orientationPortrait){
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            } else {
                setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
            }
            orientationPortrait = !orientationPortrait;
        }
        else if (id == R.id.menu_info_chart_activity) {
            FragmentManager fm = getSupportFragmentManager();
            InfoChartFragment hp = new InfoChartFragment();
            hp.show(fm, "fragment_info_chart");
        }
        return super.onOptionsItemSelected(item);
    }


    private void displayDummyChart() {
        //Dummy Chart
        Description description = new Description();
        description.setPosition(350f, 15f); //will need to change
        description.setTextColor(Color.WHITE);
        description.setTextSize(14f);
        lineChart.setDescription(description);
        lineChart.setPinchZoom(true);
        lineChart.setTouchEnabled(true);
        lineChart.getAxisRight().setDrawLabels(false);
        lineChart.getAxisRight().setEnabled(false);

        //Get values from Firebase

        xValues = Arrays.asList("1", "2", "3", "4", "5", "6", "7");

        XAxis xAxis = lineChart.getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setValueFormatter(new IndexAxisValueFormatter(xValues));
        xAxis.setLabelCount(xValues.size());
        xAxis.setGranularity(1f);
        xAxis.setTextColor(Color.WHITE);


        YAxis yAxis = lineChart.getAxisLeft();
        yAxis.setAxisMinimum(0f);
        yAxis.setAxisMaximum(100f);
        yAxis.setAxisLineWidth(2f);
        yAxis.setAxisLineColor(Color.BLACK);
        yAxis.setLabelCount(10);
        yAxis.setTextColor(Color.WHITE);

        Legend legend = lineChart.getLegend();
        legend.setTextColor(Color.WHITE);

        List <Entry> entries1 = new ArrayList<>();
        entries1.add(new Entry(0, 10f));
        entries1.add(new Entry(1, 20f));
        entries1.add(new Entry(2, 45f));
        entries1.add(new Entry(3, 66f));
        entries1.add(new Entry(4, 69f));
        entries1.add(new Entry(5, 42f));
        entries1.add(new Entry(6, 30f));

        List <Entry> entries2 = new ArrayList<>();
        entries2.add(new Entry(0, 33f));
        entries2.add(new Entry(1, 23f));
        entries2.add(new Entry(2, 43f));
        entries2.add(new Entry(3, 54f));
        entries2.add(new Entry(4, 65f));
        entries2.add(new Entry(5, 83f));
        entries2.add(new Entry(6, 24f));

        LineDataSet lineDataSet1 = new LineDataSet(entries1, "IMU1");
        lineDataSet1.setColor(Color.BLUE);
        lineDataSet1.setDrawValues(false);

        LineDataSet lineDataSet2 = new LineDataSet(entries2, "IMU2");
        lineDataSet2.setColor(Color.RED);
        lineDataSet2.setDrawValues(false);

        LineData lineData = new LineData(lineDataSet1, lineDataSet2);

        lineChart.setData(lineData);
        lineData.setValueTextColor(Color.WHITE);

        lineChart.invalidate();
    }

    private void DisplayChart (Vector<Double> rel_x){
        Description description = new Description();
        description.setText("");
        description.setPosition(350f, 15f); //will need to change
        description.setTextColor(Color.WHITE);
        description.setTextSize(0f);
        lineChart.setDescription(description);
        lineChart.setPinchZoom(true);
        lineChart.setTouchEnabled(true);
        lineChart.getAxisRight().setDrawLabels(false);
        lineChart.getAxisRight().setEnabled(false);

        //Trying stuff
        lineChart.setDrawBorders(true);


        xValues = new ArrayList<>();
        for (int i = 0; i < rel_x.size(); i++) {
            double value = i*0.1;
            String formattedValue = String.format("%.1f", value); // Limit to 1 decimal place
            xValues.add(formattedValue);
        }

        List <Entry> entries1 = new ArrayList<>();
        // Convert Vector<Double> to Vector<Float>
        Vector<Float> floatVector = new Vector<>();
        for (Double value : rel_x) {
            float floatValue = value.floatValue(); // Convert Double to Float
            floatVector.add(floatValue);
        }
        for (int i = 0; i < floatVector.size(); i++) {
            entries1.add(new Entry(i, floatVector.get(i)));
        }

        List <Entry> entries2 = new ArrayList<>();
        for (int i = 0; i < floatVector.size(); i++) {
            entries2.add(new Entry(i, 100));
        }

        List <Entry> entries3 = new ArrayList<>();
        for (int i = 0; i < floatVector.size(); i++) {
            entries3.add(new Entry(i, 110));
        }

        XAxis xAxis = lineChart.getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setValueFormatter(new IndexAxisValueFormatter(xValues));
        xAxis.setLabelCount(xValues.size());
        xAxis.setGranularity(1f);
        xAxis.setTextColor(Color.WHITE);


        YAxis yAxis = lineChart.getAxisLeft();
        yAxis.setAxisMinimum(-10f);
        yAxis.setAxisMaximum(140f);
        yAxis.setAxisLineWidth(2f);
        yAxis.setAxisLineColor(Color.BLACK);
        yAxis.setLabelCount(10);
        yAxis.setTextColor(Color.WHITE);

        //Legend legend = lineChart.getLegend();
        lineChart.getLegend().setEnabled(false);
        //legend.setTextColor(Color.WHITE);

        LineDataSet lineDataSet1 = new LineDataSet(entries1, "IMU Values");
        lineDataSet1.setColor(Color.BLUE);
        lineDataSet1.setDrawValues(false);

        LineDataSet lineDataSet2 = new LineDataSet(entries2, "range start");
        lineDataSet2.setColor(Color.RED);
        lineDataSet2.setDrawValues(false);

        LineDataSet lineDataSet3 = new LineDataSet(entries2, "range end");
        lineDataSet3.setColor(Color.RED);
        lineDataSet3.setDrawValues(false);

        LineData lineData = new LineData(lineDataSet1);
        lineChart.setData(lineData);
        lineData.setValueTextColor(Color.WHITE);

        lineChart.invalidate();

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.info_bar, menu);
        return true;
    }



    /*
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.chart_menu, menu);
        return true;
    }
    */

}

