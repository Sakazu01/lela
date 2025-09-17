// #include <iostream>
// #include <vector>
// #include <string>
// #include <sstream>

// #include "opencv2/core/core.hpp"
// #include "opencv2/flann/miniflann.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/photo/photo.hpp"
// #include "opencv2/video/video.hpp"
// #include "opencv2/features2d/features2d.hpp"
// #include "opencv2/objdetect/objdetect.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/ml/ml.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/contrib/contrib.hpp"
// #include "opencv2/core/core.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

// #include "common_type.h"
// #include "telemetry.h"
// #include "remote.h"
// #include "fire.h"

// using namespace cv;	
// using namespace std;

// //////////////////////////////////////////////////////////////////////
// /*
// DWORD WINAPI runThread(LPVOID args){
// Remote* Remoo = reinterpret_cast<Remote*>(args);
// Remoo->UpdateChannel(MODE_CH,1200);
// }

// int main()
// {

// DWORD threadId;
// int value = 10;

// hThread = CreateThread( NULL, 0, runThread, &value, 0, &threadId);
// return 0;
// }
// */
// //////////////////////////////////////////////////////////////////////

// int main( int argc, char** argv )
// {
// 	///////////////////////// INIT //////////////////////////////
	
// 	Telemetry *Tele=new Telemetry("\\\\.\\COM15");	//Connect Telemetry
// 	cout << " Telemetry\n";
// 	Remote *Remo=new Remote("COM4");				//Connect Arduino
// 	cout << " Arduino\n";
	
// 	Tele->AutoUpdate();	//Thread Telemetry (Semacam Interrupt Update)
// 	Remo->AutoSend();	//Thread Arduino (Semacam Interrupt Update)
// 	VideoCapture cap(1); //Connect To DVDriver
// 	if ( !cap.isOpened() ){  // if not success, exit program
//         cout << "Cannot open the web cam" << endl;
// 		return -1;
//     }
// 	cout << "connect kok -> Camera\n";

	
// 	//Parameter thresholding
// 	int iLowH = 0;
// 	int iHighH = 179;
// 	int iLowS = 0; 
// 	int iHighS = 255;
// 	int iLowV = 170;
// 	int iHighV = 255;
// 	//Deteksi api
// 	vector<Fire> FB;			//Objek Api
// 	vector<PseudoFire> CB;		//Objek Pseudo Api
// 	vector<PseudoFire> last_CN;	//Objek Calon Pseudo Api
// 	int CB_hist=0;		//counter
// 	//Parameter morphological filter
// 	int OpenErode=1;	
// 	int OpenDilate=5;
// 	int CloseErode=4;
// 	int CloseDilate=6; 
// 	//Parameter deteksi api
// 	int ThresTime=6;
// 	int ThresRange=50;
// 	int Target_id=-1;
// 	//Parameter kontrol gerak
// 	float KP=0.6;
// 	float KI=0.0;
// 	float KD=0.05;
// 	float EP_pitch=0;
// 	float EI_pitch=0;
// 	float ED_pitch=0;
// 	float EP_roll=0;
// 	float EI_roll=0;
// 	float ED_roll=0;
// 	float old_err_pitch=0;
// 	float old_err_roll=0;
// 	int target_last_x=0;
// 	int target_last_y=0;
// 	float Gainchan=1.0;
// 	//parameter misi
// 	int success=0;
// 	bool padam=false;
// 	bool firstframe=true;
// 	bool mission=true;
// 	bool score_it = true;
// 	int score=0;
// 	int upscore=0;
//     namedWindow("Control", CV_WINDOW_AUTOSIZE); //Control Bar Window
	
// 	//Buat Trackbar, untuk kalibrasi
// 	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
// 	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

// 	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
// 	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

// 	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
// 	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

// 	cvCreateTrackbar("ThresRange", "Control", &ThresRange, 99); 
// 	cvCreateTrackbar("ThresTime", "Control", &ThresTime, 19);

// 	cvCreateTrackbar("OpenErode", "Control", &OpenErode, 29); 
// 	cvCreateTrackbar("OpenDilate", "Control", &OpenDilate, 29);

// 	cvCreateTrackbar("CloseDilate", "Control", &CloseDilate, 29);
// 	cvCreateTrackbar("CloseErode", "Control", &CloseErode, 29);
	
// 	bool lanjut=false;
// 	while(!lanjut){
// 		if (waitKey(5) == 27){ //Tunggu press ESC, untuk abort/finish
// 			Sleep(1);
// 			lanjut=true;
// 			cout << "Hexy Love You!\n";
// 		}
// 	}
	
//     while (mission){
// 		Mat imgOriginal;	//Raw image dari kamera
//         Mat imgHSV;			//Hasil convert ke HSV
// 		Mat imgThresholded;	//Hasil Threshold
// 		Mat imgTemp;

// 		vector<vector<Point>> contours;	//contour terdeteksi.
// 		vector<Vec4i> hierarchy;		//hierarchy contour.
// 		vector<PseudoFire> CN;			//Objek mungkin api pada frame ini.
		
// 		if(firstframe){	//Prosedur Auto Take Off
// 			cout<<"Take Off\n";
// 			int power=1500;
// 			for(int i=0;i<500;i++){	//power up
// 				Remo->UpdateChannel(THR_CH,power++);
// 				Sleep(5);
// 			}
// 			Sleep(3000);	//climbing state
// 			Remo->UpdateChannel(PITCH_CH,2000);	//pitch down.
// 			cout<<"Pitch Down\n";
// 			Sleep(3000);	//approach Area B
// 			Remo->UpdateChannel(THR_CH,1500);	//Throttle level.
// 			cout<<"Throttle Mid\n";
// 			Sleep(14500);
// 			Remo->UpdateChannel(PITCH_CH,1500);	//Pitch level.
// 			cout<<"Approach done!\n";
// 		}
		
// 		bool bSuccess = cap.read(imgOriginal); //read frame

//         if (!bSuccess){
//              cout << "Read frame gagal.\n";
//              break;
//         }

// 		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Konversi gambar ke HSV color system
		
// 		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Thresholding
		
// 		//morphological opening (menghapus objek yang terlalu kecil)
// 		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(OpenErode, OpenErode)));
// 		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(OpenDilate, OpenDilate)) ); 

// 		//morphological closing (menggabungkan objek2 yang berdekatan)
// 		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(CloseDilate, CloseDilate)) ); 
// 		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(CloseErode, CloseErode)) );

// 		imgThresholded.copyTo(imgTemp);
// 		findContours(imgTemp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);	//Contour detection
// 		if(hierarchy.size()>0){
// 			int numObj=hierarchy.size();
// 			for(int i=0;i>=0;i=hierarchy[i][0]){
// 				Moments moment=moments((Mat)contours[i]);	//Pusat momen dari contour digunakan sebagai koordinat
// 				double area=moment.m00;
// 				PseudoFire tempo;
// 				tempo.x=moment.m10/area;
// 				tempo.y=moment.m01/area;
// 				tempo.id=0;
// 				tempo.flag=0;
// 				tempo.val=0;
// 				CN.push_back(tempo);	//contour yang ditemukan dimasukkan kedalam CN.
// 			}
// 		}

// 		if(CN.size()>100){	//Jika jumlah CN>4, maka dianggap terlalu noisy.
// 			if(!firstframe){
// 				CN=last_CN;	//Pakai data CN dari frame sebelumnya.
// 			}
// 		}else{
// 			last_CN=CN;	//Update data CN.
// 		}

// 		///////FIRE DETECTION/////////
// 		//untuk semua CB; untuk semua CN; kalau ada CN yang dekat dengan CB, update CB, lalu flag CN. 
// 		//kalau CN habis, CB yang belum diupdate val--. kalau CB udah habis, sisa CN masukkan ke CB.
		
// 		for(int i=0;i<CB.size();i++){	//Update CB, CB yang gak keupdate, val--
// 			for(int j=0;j<CN.size();j++){
// 				if((CB[i].x-CN[j].x)*(CB[i].x-CN[j].x)+(CB[i].y-CN[j].y)*(CB[i].y-CN[j].y)<ThresRange*ThresRange && CN[j].flag==0){
// 					CB[i].x=CN[j].x;
// 					CB[i].y=CN[j].y;
// 					CN[j].flag=1;
// 					CB[i].flag=1;
// 					if(CB[i].val<ThresTime){
// 						CB[i].val++;
// 					}	
// 				}
// 			}
// 			if(CB[i].flag==0){
// 				CB[i].val--;
// 			}
// 			CB[i].flag=0;
// 		}

// 		int itera=0;
// 		while(itera<CB.size()){	//CB yang valnya 0, dimatikan.
// 			if(CB[itera].val<=0){
// 				CB.erase(CB.begin()+itera);
// 				itera=0;
// 			}else{
// 				itera++;
// 			}
// 		}

// 		/*
// 		// Alternatif Erasing
// 		for(int i=0;i<CB.size();i++){
// 			if(CB[i].val==0){
// 				CB.erase(CB.begin()+i,CB.begin()+i+1);
// 				i--;
// 			}
// 		}
// 		*/

// 		for(int i=0;i<CN.size();i++){	//CN yang tidak mengupdate CB, dimasukkan ke CB.
// 			if(CN[i].flag==0){
// 				CN[i].id=CB_hist++;
// 				CN[i].val=1;
// 				CN[i].flag=0;
// 				CB.push_back(CN[i]);
// 			}
// 		}

// 		//FB yang punya afiliasi CB, update posisinya.
// 		for(int i=0;i<FB.size();i++){
// 			for(int j=0;j<CB.size();j++){
// 				if(FB[i].GetAff()==CB[j].id && CB[j].flag==0 && FB[i].GetFlag()==0){
// 					FB[i].UpdateLoc(CB[j].x,CB[j].y);
// 					CB[j].flag=1;
// 					FB[i].SetFlag(1);
// 				}
// 			}
// 		}

// 		//Hapus api yang tidak punya CB.
// 		itera=0;
// 		while(itera<FB.size()){
// 			if(FB[itera].GetFlag()==0){
// 				FB.erase(FB.begin()+itera);
// 				itera=0;
// 			}else{
// 				itera++;
// 			}
// 		}

// 		//CB yang tidak punya api dan usianya cukup, dibuatkan api.
// 		for(int i=0;i<CB.size();i++){
// 			if(CB[i].flag==0 && CB[i].val>=ThresTime){
// 				Fire a= Fire(CB[i].x,CB[i].y,CB[i].id);
// 				FB.push_back(a);
// 			}
// 			CB[i].flag=0;
// 		}

// 		for(int i=0;i<FB.size();i++){
// 			FB[i].SetFlag(0);
// 		}

// 		//////////////////////// DECISION TAKING ///////////////////////
// 		//Api di posisi.
// 		/*
// 		if(Tele->GetAlti()<=200){
// 			Gainchan=0.8;
// 		}else{
// 			Gainchan=1;
// 		}
// 		*/
// 		//PASANG INTERRUPT 5 MENIT, LALU PULANG
// 		if(score_it && Tele->GetAlti()<150){
// 			score++;
// 		}
// 		if(padam){	//kalau api dalam keadaan padam.
// 			padam=false;
// 			switch (success){
// 			case 1:	//Asumsi titik B sudah padam.
// 					Remo->UpdateChannel(PITCH_CH,2000);
// 					Remo->UpdateChannel(THR_CH,2000);
// 					Sleep(6000);
// 					Remo->UpdateChannel(THR_CH,1500);
// 					Sleep(6000);
// 					Remo->UpdateChannel(PITCH_CH,1500);
// 					//Lanjutkan Cari Api di B.
// 					break;
// 			case 2:	//Asumsi titik A sudah padam.
// 					Remo->UpdateChannel(THR_CH,2000);
// 					Remo->UpdateChannel(PITCH_CH,1000);
// 					Sleep(6000);
// 					Remo->UpdateChannel(THR_CH,1500);
// 					Sleep(50000);
// 					Remo->UpdateChannel(PITCH_CH,1500);
// 					//Lanjutkan Cari Api di C.
// 					break;
// 			case 3:	//Asumsi titik C sudah padam.
// 					Remo->UpdateChannel(MODE_CH,1200);
// 					mission=false;
// 					//Pulang
// 					break;
// 			}
// 		}
// 		if(Target_id!=-1){	//Kalau ada target
// 			int xpos,ypos;
// 			bool ada=false;
// 			for(int i=0;i<FB.size();i++){	//Cek apakah target masih ada di list api.
// 				if(FB[i].GetId() ==Target_id){
// 					ada=true;
// 					xpos=FB[i].GetX();
// 					ypos=FB[i].GetY();
// 					target_last_x=xpos; 
// 					target_last_y=ypos;
// 				}
// 			}
// 			if(!ada){	//Kalau target tidak ada dalam list api.
// 				Remo->UpdateChannel(THR_CH,1950);	//naik
// 				upscore++;
// 				Target_id=-1;	//target invalid, tapi akan  segera menemukan target baru.
				
// 				/*
// 				Remo->UpdateChannel(PITCH_CH,1500);
// 				Remo->UpdateChannel(ROLL_CH,1500);
// 				*/
// 				//kalau ada target, tapi ternyata ilang. (mungkin padam, mungkin out of range)
// 			}else{	//Kalau target ada dalam list api (Target terlihat)
// 				float err_pitch=(240-ypos);	//actual error
// 				float err_roll=(320-xpos);
// 				if(abs(err_pitch)<25)err_pitch=0;	//deadband
// 				if(abs(err_roll)<25)err_roll=0;
// 				if(abs(err_roll)<70 && abs(err_pitch)<70 && Tele->GetAlti()>140){//&& Tele->GetAlti()>=70){	//Drop Radius
// 					Remo->UpdateChannel(THR_CH,1000);	//Drop
// 					if(score>1000){
// 						Remo->UpdateChannel(MODE_CH,1200);
// 					}
// 				}else{
// 					Remo->UpdateChannel(THR_CH,1505);	//Level
// 				}
// 				EP_pitch=err_pitch;	//Proportional Error
// 				EI_pitch+=err_pitch;	//Integral Error
// 				ED_pitch=err_pitch-old_err_pitch;	//Derivative Error
// 				EP_roll=err_roll;
// 				EI_roll+=err_roll;
// 				ED_roll=err_roll-old_err_roll;
// 				old_err_roll=err_roll;	//update old error
// 				old_err_pitch=err_pitch;
// 				int cor_pitch=(int)((KP*EP_pitch+KI*EI_pitch+KD*ED_pitch)*Gainchan);	//Correction
// 				int cor_roll=(int)((KP*EP_roll+KI*EI_roll+KD*ED_roll)*Gainchan);
// 				int newPitch=Remo->GetChannel(PITCH_CH)+cor_pitch;
// 				int newRoll=Remo->GetChannel(ROLL_CH)+cor_roll;
// 				if(newPitch<1044) newPitch=1044;
// 				if(newPitch>1956) newPitch=1956;
// 				if(newRoll<1044) newRoll=1044;
// 				if(newRoll>1956) newRoll=1956;
// 				Remo->UpdateChannel(PITCH_CH,newPitch);
// 				Remo->UpdateChannel(ROLL_CH,newRoll);
// 				//cout<< Remo->GetChannel(PITCH_CH) << " " << Remo->GetChannel(ROLL_CH) << endl;
// 			}
// 		}else{	//Kalau tidak ada target
// 			upscore++;
// 			Remo->UpdateChannel(THR_CH,20000);	//Naik
// 			if(FB.size()>0){	//Kalau ada api
// 				Target_id=FB[0].GetId();	//Jadikan target.
// 			}
// 		}
		
// 		if(upscore>=700){
// 			cout<<"Pulang!\n";
// 			Remo->UpdateChannel(MODE_CH,1200);
// 		}

// 		for(int i=0;i<FB.size();i++){	//Untuk semua api, Buat lingkaran
// 			if(FB[i].GetId()==Target_id){
// 				circle(imgOriginal,Point(FB[i].GetX(),FB[i].GetY()),5,Scalar(100,100,255),2);	// Untuk target, warnanya beda.
// 			}else{
// 				circle(imgOriginal,Point(FB[i].GetX(),FB[i].GetY()),5,Scalar(0,255,0),2);
// 			}
// 		}

// 		circle(imgOriginal,Point(640/2,480/2),50,Scalar(255,0,255),3);	//Lingkaran tengah

// 		for(int i=0;i<CB.size();i++){	//Untuk semua CB buat lingkaran
// 			circle(imgOriginal,Point(CB[i].x,CB[i].y),10,Scalar(0,0,255),2);
// 		}

// 		for(int i=0;i<CN.size();i++){	//Untuk semua CN buat lingkaran
// 			circle(imgOriginal,Point(CN[i].x,CN[i].y),15,Scalar(255,0,0),2);
// 		}

// 		imshow("Thresholded Image", imgThresholded); //tampilkan gambar threshold (untuk memeriksa efektifitas filter)
// 		imshow("Original", imgOriginal); //tampilkan gambar original + lingkaran2.

//         if (waitKey(1) == 27){ //Tunggu press ESC, untuk abort/finish
// 			Tele->~Telemetry();
// 			Remo->~Remote();
// 			cout << "ESC pressed. Good Luck\n";
//             break; 
// 		}
// 		firstframe=false;
// 	}
// 	return 0;
// }