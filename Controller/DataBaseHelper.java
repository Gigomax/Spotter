package com.example.spotter.Controller;

import android.content.Context;
import android.content.ContentValues;
import android.database.Cursor;
import android.database.sqlite.SQLiteOpenHelper;
import android.database.sqlite.SQLiteDatabase;
import android.util.Log;
import androidx.annotation.Nullable;

import com.example.spotter.Model.FlexSensor;
import com.example.spotter.Model.ImuSensor;

import java.util.Vector;


public class DataBaseHelper extends SQLiteOpenHelper{
    //LOGCAT Variable
    static final String Lobster = "Lobster_DataBaseHelper";
    //Static Variables
    public static final String IMU_TABLE = "IMU_TABLE";
    public static final String FLEX_TABLE = "FLEX_TABLE";
    public static final String COLUMN_ANGLE_1_X = "ANGLE_1_X";
    public static final String COLUMN_ANGLE_2_X = "ANGLE_2_X";
    public static final String COLUMN_ANGLE_1_Y = "ANGLE_1_Y";
    public static final String COLUMN_ANGLE_2_Y = "ANGLE_2_Y";
    public static final String COLUMN_RELATIVE_X = "RELATIVE_X";
    public static final String COLUMN_RELATIVE_Y = "RELATIVE_Y";
    public static final String COLUMN_FLEX = "FLEX";
    public static final String COLUMN_ACTIVITY = "ACTIVITY";
    public static final String ACQUISITION_ID = "ACQUISITION_ID";

    public DataBaseHelper(@Nullable Context context){
        super(context, "sensorValues.db", null, 1);}

    public void DeleteDatabase(Context context){
        context.deleteDatabase("sensorValues.db");
    } //if need to erase db
    @Override
    public void onCreate(SQLiteDatabase db) {


        //Table for IMU
        db.execSQL("CREATE TABLE " + IMU_TABLE + " ("+ ACQUISITION_ID + " INTEGER, " + COLUMN_ANGLE_1_X + " DOUBLE, " + COLUMN_ANGLE_1_Y + " DOUBLE, " + COLUMN_ANGLE_2_X + " DOUBLE, " + COLUMN_ANGLE_2_Y + " DOUBLE, " + COLUMN_RELATIVE_X + " DOUBLE, " + COLUMN_RELATIVE_Y + " DOUBLE, " + COLUMN_ACTIVITY + " TEXT ) ");
        //Table for Flex
        db.execSQL("CREATE TABLE " + FLEX_TABLE + " (" + ACQUISITION_ID + " INTEGER, " + COLUMN_FLEX + " DOUBLE, " + COLUMN_ACTIVITY + " TEXT ) ");

    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {

        // Drop the existing tables if they exist
        db.execSQL("DROP TABLE IF EXISTS " + IMU_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + FLEX_TABLE);

        // Create new tables with updated schema
        onCreate(db);

    }

    //Methods
    public boolean insertSensors(FlexSensor flex, ImuSensor imu, String activity, int acq_id){ // for inserting both flex and imu sensors to database

        SQLiteDatabase db = this.getWritableDatabase();
        ContentValues content = new ContentValues();

        try{

            //IMU sensor table input
            content.put(COLUMN_ANGLE_1_X, imu.getAngle1_x());
            content.put(COLUMN_ANGLE_1_Y, imu.getAngle1_y());
            content.put(COLUMN_ANGLE_2_X, imu.getAngle2_x());
            content.put(COLUMN_ANGLE_2_Y, imu.getAngle2_y());
            content.put(COLUMN_RELATIVE_X, imu.getRelative_x());
            content.put(COLUMN_RELATIVE_Y, imu.getRelative_y());
            content.put(COLUMN_ACTIVITY, activity);
            content.put(ACQUISITION_ID, acq_id);
            db.insert(IMU_TABLE, null, content);
            Log.d(Lobster, "IMU Sensor: " + content.toString());
            content.clear();

            //Flex sensor table input
            content.put(COLUMN_FLEX, flex.getFlex());
            content.put(COLUMN_ACTIVITY, activity);
            content.put(ACQUISITION_ID, acq_id);
            db.insert(FLEX_TABLE, null, content);
            Log.d(Lobster, "Flex Sensor: " + content.toString());
            content.clear();

        }
        catch(Exception e){

            Log.e(Lobster, "Error on Insert Sensors for Database: " + e.getMessage());
            return false;
        }

        return true;
    }
    public boolean insertFlex(FlexSensor flex, String activity, int acq_id){ // for inserting only flex sensor to database

        SQLiteDatabase db = this.getWritableDatabase();
        ContentValues content = new ContentValues();

        try{

            content.put(COLUMN_FLEX, flex.getFlex());
            content.put(COLUMN_ACTIVITY, activity);
            content.put(ACQUISITION_ID, acq_id);
            db.insert(FLEX_TABLE, null, content);
            Log.d(Lobster, "Flex Sensor: " + content.toString());
            content.clear();
        }
        catch(Exception e){
            Log.e(Lobster, "Error on Flex Sensor Insert for Database: " + e.getMessage());
            return false;
        }

        return true;
    }
    public boolean insertImu(ImuSensor imu, String activity, int acq_id){// for inserting only imu sensors to database

        SQLiteDatabase db = this.getWritableDatabase();
        ContentValues content = new ContentValues();

        try{

            content.put(COLUMN_ANGLE_1_X, imu.getAngle1_x());
            content.put(COLUMN_ANGLE_1_Y, imu.getAngle1_y());
            content.put(COLUMN_ANGLE_2_X, imu.getAngle2_x());
            content.put(COLUMN_ANGLE_2_Y, imu.getAngle2_y());
            content.put(COLUMN_RELATIVE_X, imu.getRelative_x());
            content.put(COLUMN_RELATIVE_Y, imu.getRelative_y());
            content.put(COLUMN_ACTIVITY, activity);
            content.put(ACQUISITION_ID, acq_id);
            db.insert(IMU_TABLE, null, content);
            Log.d(Lobster, "IMU Sensor: " + content.toString());
            content.clear();
        }
        catch(Exception e){

            Log.e(Lobster, "Error on IMU Sensors Insert for Database: " + e.getMessage());
            return false;
        }

        return true;
    }
    public Vector<Double[]> getIMU(){ // get IMU data for chart

        Vector<Double[]> vals = new Vector<>();
        Vector<Double> valX = new Vector<>();
        Vector<Double> valY = new Vector<>();
        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor;
        String[] angles = {COLUMN_RELATIVE_X, COLUMN_RELATIVE_Y};


        //Get table values
        cursor = db.query(IMU_TABLE, angles, null, null, null, null, null);
        if(cursor.moveToFirst()){
            do{

                Double[] relatives = new Double[2]; // using 2x1 array to store
                double relativeX = cursor.getDouble(0); relatives[0] = relativeX;
                double relativeY = cursor.getDouble(1); relatives[1] = relativeY;
                Log.d(Lobster, "Relative angles are: X-axis = " + relatives[0] + " Y-axis = " + relatives[1]);
                vals.add(relatives);
                valX.add(relativeX);
                valY.add(relativeY);

            }while(cursor.moveToNext());

        }

        cursor.close();
        db.close();
        return vals;
    }

    public Vector<Double> getFlex() { // get flex data for chart

        Vector<Double> val = new Vector<>();
        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor;

        cursor = db.query(FLEX_TABLE, new String[]{COLUMN_FLEX}, null, null, null, null, null);
        if(cursor.moveToFirst()) {
            do {

                double flex = cursor.getDouble(0);
                Log.d(Lobster, "Flex angle: " + flex);
                val.add(flex);

            } while (cursor.moveToNext());
        }

        cursor.close();
        db.close();
        return val;
    }

    public boolean isDatabaseEmpty() {
        SQLiteDatabase db = this.getReadableDatabase();

        // Check if IMU_TABLE is empty
        Cursor imuCursor = db.rawQuery("SELECT * FROM " + IMU_TABLE, null);
        int nb = imuCursor.getCount();
        boolean isImuTableEmpty = imuCursor.getCount() == 0;
        imuCursor.close();

        // Check if FLEX_TABLE is empty
        Cursor flexCursor = db.rawQuery("SELECT * FROM " + FLEX_TABLE, null);
        int nb2 = flexCursor.getCount();
        boolean isFlexTableEmpty = flexCursor.getCount() == 0;
        flexCursor.close();

        db.close();

        // Return true if both tables are empty
        return isImuTableEmpty && isFlexTableEmpty;
    }

    public Vector<Double> getIMU_Relative_Angle(int acquisitionId, String activity) {
        Vector<Double> valX = new Vector<>();
        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor;
        String[] angles = {COLUMN_RELATIVE_X};

        // Define the WHERE clause to filter by ACQUISITION_ID and COLUMN_ACTIVITY
        String whereClause = ACQUISITION_ID + " = ? AND " + COLUMN_ACTIVITY + " = ?";
        String[] whereArgs = {String.valueOf(acquisitionId), activity};

        // Get table values with the specified filters
        cursor = db.query(IMU_TABLE, angles, whereClause, whereArgs, null, null, null);
        if (cursor.moveToFirst()) {
            do {
                double relativeX = cursor.getDouble(0);
                Log.d(Lobster, "Relative angle X: " + relativeX);
                valX.add(relativeX);
            } while (cursor.moveToNext());
        }

        cursor.close();
        db.close();
        return valX;
    }

}
