package frc.robot.util.AprilTagStuff;





import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;

public class AprilTagDataLoader {


   public static AprilTagInfo[] aprilTags;
   public static FieldInfo info;


//   public static void LoadAprilTagDataFromJSON(){

//     try{
//         AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/fielddata/aprilTagInfo.json");

//         Optional<Pose3d> thing = layout.getTagPose(1);
//         System.out.println("OEIPJFSOIJF:OISHEFO:IHSEIOU:FHISUEHFLIUHSEF==================== " + thing.get().getX());

//     } catch (Exception e){
//         System.out.println("FFFFFFFFFFAAAAAAAAAAAAAAAIIIIIIIIIIIILLLLLLLLLLLLEEEEEEEEEEEEEEEDDDDDDDDDDDDDDD");
//         System.out.println(e);
//     }
//     }

    public static void LoadAprilTagDataFromJSON(){
        try {

            BufferedReader br = new BufferedReader(new FileReader(new File(
                    Filesystem.getDeployDirectory() + "/fielddata/aprilTagInfo.json"
            )));

            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();


            Gson gson = new Gson();

            info = gson.fromJson(fileContent, FieldInfo.class);

            aprilTags = info.fiducials;


        } catch (Exception e){ System.out.println("Bad!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " + e.toString());}
    }
}
