/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.main;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.multi.qrcode.QRCodeMultiReader;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;

public class Main_activity extends Activity implements IOIOLooperProvider, SensorEventListener,
		CameraBridgeViewBase.CvCameraViewListener2  // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity

	boolean redRobot = true; //account for different gear ratios
	int forwardSpeed;
	int turningSpeed;
	int obstacleTurningSpeed;

	//useful//////////////////////////////////////
	float irLeft;
	float irCenter;
	float irRight;
	int counter_for_stop=0;//stop for scanning
	int turning_counter=0; //turning cycles
	////////////////////////////////////////
	// ioio variables
	IOIO_thread_rover_4wd m_ioio_thread;

	//variable for qrcode
	private static boolean isZxingThreadRunning;
	int qrCode;

	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetector;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;

	//app state variables
	private boolean autoMode;

	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravityV;
	float[] mGyro;

	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mGravity;
	float[] mGeomagnetic;
	public float heading;
	public float bearing;
	public float desiredHeading;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;
	TextView desiredHeadingText;


	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}

	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);


		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);

		helper_.create(); // from IOIOActivity

		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);
		desiredHeadingText = (TextView) findViewById(R.id.distanceText);

		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new View.OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
				}
			}
		});

		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}

		//set up compass
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		mCompass= mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		mAccelerometer= mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		mGravityS = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);


		//set speeds. adjust accordingly for your robot
		if(redRobot){
			forwardSpeed = 180;
			turningSpeed = 100;
			obstacleTurningSpeed = 100;
		} else {
			forwardSpeed = 110 ;
			turningSpeed = 80;
			obstacleTurningSpeed = 90;
		}

	}

	@Override
	public final void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
	}

	//Called whenever the value of a sensor changes
	@Override
	public final void onSensorChanged(SensorEvent event) {
		if (m_ioio_thread != null) {
			irLeft=m_ioio_thread.get_ir1_reading();
			irCenter=m_ioio_thread.get_ir2_reading();
			irRight=m_ioio_thread.get_ir3_reading();
			setText("sonar1: " + irLeft, sonar1Text);
			setText("sonar2: " + irCenter, sonar2Text);
			setText("sonar3: " +irRight, sonar3Text);
			setText("bearing: " + bearing, bearingText);
			setText("heading: " + heading, headingText);
			setText("desired heading" + desiredHeading, desiredHeadingText);


			if (event.sensor.getType() == Sensor.TYPE_GRAVITY)
				mGravityV = event.values;
			if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
				mGyro = event.values;
			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
				mGravity = event.values;
			if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
				mGeomagnetic = event.values;
			if (mGravity != null && mGeomagnetic != null) {
				float[] temp = new float[9];
				float[] R = new float[9];
				//Load rotation matrix into R
				SensorManager.getRotationMatrix(temp, null, mGravity, mGeomagnetic);
				//Remap to camera's point-of-view
				SensorManager.remapCoordinateSystem(temp, SensorManager.AXIS_X, SensorManager.AXIS_Z, R);
				//Return the orientation values
				float[] values = new float[3];
				SensorManager.getOrientation(R, values);
				//Convert to degrees
				for (int i = 0; i < values.length; i++) {
					Double degrees = (values[i] * 180) / Math.PI;
					values[i] = degrees.floatValue();
				}
				//Update the compass direction
				heading = values[0] + 12;
				heading = (heading * 5 + fixWraparound(values[0] + 12)) / 6; //add 12 to make up for declination in Irvine, average out from previous 2 for smoothness
				bearing = desiredHeading - heading;
			}

		}
	}

	//Scan for QR code and save information to phone
	public void zxing() throws ChecksumException, FormatException {

		Bitmap bMap = Bitmap.createBitmap(mRgba.width(), mRgba.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(mRgba, bMap);
		int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
		//copy pixel data from the Bitmap into the 'intArray' array
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeMultiReader();

		try {
			Result result = reader.decode(bitmap);
			//Log.i(TAG, "CODE FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			// Log.i(TAG, result.getText());
			if (result.getText().equals("START")) {
				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						qrCode=1;


					}
				});
			} else if (result.getText().equals("STOP")) {
				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						qrCode=2;
					}
				});
			}
			else if (result.getText().equals("TURN")) {
				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						qrCode=3;

					}
				});}
		} catch (NotFoundException e) {
			// Log.i(TAG, "Code Not Found");
			e.printStackTrace();
		}
	}


	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
		mSensorManager.registerListener(this, mCompass, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mGravityS, SensorManager.SENSOR_DELAY_NORMAL);
	}

	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		mSensorManager.unregisterListener(this);
	}


	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle","main activity restarting");
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		//To set color, find   values of desired color and convert each value to 1-255 scale
		//mDetector.setHsvColor(new Scalar(7, 196, 144)); // red
		mDetector.setHsvColor(new Scalar(70.25,240,35 ));
	}
	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

		mRgba = inputFrame.rgba();
		mDetector.process(mRgba);

		List<MatOfPoint> contours = mDetector.getContours();
		// Log.e("rescue robotics", "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);


		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);


		Thread zxingThread = new Thread() {
			@Override
			public void run() {
				if (isZxingThreadRunning) {
					return;
				}

				isZxingThreadRunning = true;
				try {
					zxing();
				} catch (ChecksumException e) {
					e.printStackTrace();
				} catch (FormatException e) {
					e.printStackTrace();
				}
				isZxingThreadRunning = false;
			}
		};

		zxingThread.start();

		if (autoMode && qrCode == 1) {//now we start to move, change to mode 4
			m_ioio_thread.set_speed(1500+forwardSpeed);
			m_ioio_thread.set_steering(1500);
			desiredHeading = heading;//useless now


			qrCode = 4;
			//Log.i(TAG, "start works!!!");
		}
		else if(autoMode && mDetector.blobsDetected() == 0 && qrCode==4) {
			//in mode 4, if what we see is not a paper, we will use IR. we still need to set up mDetector to be the color of  paper
			m_ioio_thread.set_speed(1500+forwardSpeed);
			m_ioio_thread.set_steering(1500);

			if (irCenter > 17) {//not close enough to a obstacle, maybe not 17
				m_ioio_thread.set_speed(1800);
				m_ioio_thread.set_steering(1500);
			} else {//mid is too close to the obstacle
				m_ioio_thread.set_steering(1200); //should we turn left or should we move back??
			}

			if (irLeft < 17) {//left IR feel obstacle,turn right
				m_ioio_thread.set_steering(1800);
			}

			if (irRight < 17) {//right IR feel obstacle, turn left
				m_ioio_thread.set_steering(1200);
			}

		}

		else if(autoMode && mDetector.blobsDetected() > 0 &&  qrCode==4) {
			//if we see the paper(must be in the middle), we need to stop
			//we stop 10 cycle, if no new command, we will continue to move
			//only in moving mode, we need to stop when scanning, so we don't add this part of code to qrcode 3(turning)

			//my be need to add   mDetector.getMomentX()<=25&& mDetector.getMomentX()>=-25&&  to condition, i am not sure
			Log.d("A debug variable","desired heading" +Float.toString(desiredHeading) );//useless line

			counter_for_stop++;
			if (counter_for_stop<60){
				m_ioio_thread.set_speed(1500);
				m_ioio_thread.set_steering(1500);
			}
			else{
				m_ioio_thread.set_speed(1800);
				m_ioio_thread.set_steering(1500);
				counter_for_stop=0;       //added counter reset
			}

		}
		else if (autoMode && qrCode == 2) {
			m_ioio_thread.set_speed(1500);
			m_ioio_thread.set_steering(1500);
			// Log.i(TAG, "stop works!!!");
		}
		else if (autoMode && qrCode == 3) {
			turning_counter++;
			if (turning_counter<30){//set a counter, turn 180 degree (still need to figure out how many cycles it need )
				m_ioio_thread.set_steering(1500+turningSpeed);
			}
			else{
				qrCode=4;//if it turns 180, we will change to move mode(qrcode 4), which means we will move straight
				turning_counter=0;
			}


//    // Log.i(TAG, "turn works!!!");
//       if (mDetector.blobsDetected() > 0&& m_ioio_thread.get_ir2_reading() < 25&& m_ioio_thread != null) {
//    //    Log.i(TAG, " blob detected!!!");
//          double momentX = mDetector.getMomentX();
//          //BUCKET ON RIGHT SIDE.
//          if(momentX > 25) {
//             m_ioio_thread.set_speed(1500+forwardSpeed);
//             m_ioio_thread.set_steering(1500-turningSpeed);
//    //       Log.i(TAG, "momentX>25");
//          }
//          //BUCKET ON LEFT SIDE
//          else if(momentX<-25) {
//             m_ioio_thread.set_speed(1500+forwardSpeed);
//             m_ioio_thread.set_steering(1500+turningSpeed);
//    //       Log.i(TAG, "momentX<-25");
//          }
//          //BUCKET NEAR MIDDLE
//          else {
//             m_ioio_thread.set_speed(1500+forwardSpeed);
//             m_ioio_thread.set_steering(1500-turningSpeed);
//    //       Log.i(TAG, "in between 25 and -25");
//          }
//       }
//       else {
//          if(bearing>0 && bearing<=180) {
//             m_ioio_thread.set_speed(1500 + forwardSpeed);
//             m_ioio_thread.set_steering(1500-turningSpeed);
//    //       Log.i(TAG, "turn left");
//          }
//          else if (bearing<0 || bearing>180) {
//             m_ioio_thread.set_speed(1500 + forwardSpeed);
//             m_ioio_thread.set_steering(1500+turningSpeed);
//    //       Log.i(TAG, "turn right");
//          }
//          else {
//             m_ioio_thread.set_speed(1500 + forwardSpeed);
//             m_ioio_thread.set_steering(1500);
//    //       Log.i(TAG, "straight");
//          }
//       }
		}
		else {
			//    Log.i(TAG, "no code.");
		}
		return mRgba;
	}

	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;

	}

	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}

	//set the text of any text view in this application
	public void setText(final String str, final TextView tv)
	{
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				tv.setText(str);
			}
		});
	}

	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 *

	 * */

	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
				.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread_rover_4wd();
			return m_ioio_thread;
		} else
			return null;
	}


	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
		if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}
}

