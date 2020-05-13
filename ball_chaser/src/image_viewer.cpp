
    //Look at only Red (white has 255 in all colors)
    for(int i =0; i < img.height*img.step; i += 3){
        //img.height*img.step
        if (img_R[i] < 255){
            img_R[i] = 0;
            }
        else{
            bin_x[i%img.height] += 1;
        }
        
    }

    //sum binned xaxis data:
    // if not white pixels added ==0, then stop
    for (int i =0; i< bin_x.size(); i++){
        bin_x_sum += bin_x[i];
    }
    if (bin_x_sum == 0){
        //No white ball:  STOP
        ss = "No Ball";
        ROS_INFO_STREAM(ss);
        }
    else{
        
        CM = center_of_mass(bin_x);
        for (int i =0; i< bin_x.size(); i++){
            if (bin_x[i]>0){
                ROS_INFO_STREAM(to_string(i));
            }
        }
        ss = "Bined x_Axis:  " +to_string(bin_x.size()) + "   / img.width: " + to_string(img.width) + "  / img heigt: " + to_string(img.height) + "  step "+ to_string(img.step)+ " // Center of mass" + to_string(CM);
        ROS_INFO_STREAM(ss);
        }