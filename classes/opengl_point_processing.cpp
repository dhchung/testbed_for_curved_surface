#include "opengl_point_processing.h"


OpenglPointProcessing::OpenglPointProcessing(std::string window_name){
    w_name = window_name;
    screenWidth = 1600;
    screenHeight = 900;

    camera = new Camera(glm::vec3(-2.0f, 0.0f, 1.0f), glm::vec3(0.0f,0.0f,-1.0f));

    lastX = (float)screenWidth/2.0f;
    lastY = (float)screenHeight/2.0f;
    firstMouse = true;

    deltaTime = 0.0f;
    lastFrame = 0.0f;
    

    imgs.clear();
}

OpenglPointProcessing::~OpenglPointProcessing(){

}

void OpenglPointProcessing::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl

    
    window = glfwCreateWindow(screenWidth,screenHeight, w_name.c_str(), NULL, NULL);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetWindowUserPointer(window, this);


    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    

    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { //glfwGetProcAddress : Get correct functions regarding to OS or compile environment
        std::cout<< "Failed to initialize GLAD" << std::endl;
    }

    point_shader.InitShader("shaders/point/vertex_shader.vs", "shaders/point/fragment_shader.fs");
    plane_shader.InitShader("shaders/plane/with_texture/vertex_shader.vs", "shaders/plane/with_texture/fragment_shader.fs");

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenVertexArrays(1, &VAO);
    glEnable(GL_DEPTH_TEST);  

}

void OpenglPointProcessing::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglPointProcessing::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

void OpenglPointProcessing::plot_3d_points(PointCloud & pt_cld){


    int data_size = pt_cld.point_cloud.cols();

    float vertices[6*data_size];
   
    for(int i = 0; i<data_size; ++i){
        for(int j = 0; j<3; ++j){
            vertices[i*6+j] = pt_cld.point_cloud(j,i);
        }
        for(int j = 3; j<6; ++j){
            vertices[i*6+j] = pt_cld.point_color(j-3,i)/255.0f;
        }
    }

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    //Model and camera;

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);

    projection = glm::perspective(glm::radians(camera->Zoom),
                                    float(screenWidth)/float(screenHeight), 
                                    0.1f, 
                                    100.0f);

    view = glm::lookAt(glm::vec3(-1.0f, -1.0f, -1.0f),
                       glm::vec3(1.0f, 0.0f, 0.0f),
                       glm::vec3(0.0f, 0.0f, -1.0f));

    clear_window();
    point_shader.use();
    point_shader.setMat4("model", model);
    point_shader.setMat4("view", view);
    point_shader.setMat4("projection", projection);
    
    glBindVertexArray(VAO);
    glPointSize(3.0);
    glDrawArrays(GL_POINTS, 0, data_size);
    glBindVertexArray(0);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

glm::mat4 OpenglPointProcessing::eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4){

    glm::mat4 result;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            result[j][i] = e_mat4(i,j);
        }
    }
    return result;
}


void OpenglPointProcessing::plot_global_points(std::vector<PointCloud> & g_pt_cld,
                                               std::vector<float> & state,
                                               int & idx){

    clear_window();

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);


    view = glm::lookAt(glm::vec3(state[0]-2, state[1]+2, 1),
                    glm::vec3(state[0]+1, state[1], state[2]),
                    glm::vec3(0.0f, 0.0f, -1.0f));


    for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

        int data_size = g_pt_cld[data_id].point_cloud.cols();

        float vertices[6*data_size];
    
        for(int i = 0; i<data_size; ++i){
            for(int j = 0; j<3; ++j){
                vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i);
            }
            for(int j = 3; j<6; ++j){
                vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
            }
        }

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);


        //Model and camera;


        Eigen::Matrix4f cur_state;
        
        // c_trans.xyzrpy2t(g_pt_cld[data_id].state, &cur_state);

        model = eigen_mat4_to_glm_mat4(g_pt_cld[data_id].state_matrix);

        projection = glm::perspective(glm::radians(45.0f),
                                        float(screenWidth)/float(screenHeight), 
                                        0.1f, 
                                        100.0f);



        point_shader.use();
        point_shader.setMat4("model", model);
        point_shader.setMat4("view", view);
        point_shader.setMat4("projection", projection);
        
        glBindVertexArray(VAO);
        glPointSize(3.0);
        glDrawArrays(GL_POINTS, 0, data_size);
        glBindVertexArray(0);
        draw_axis(1.0f, 20.0f, &point_shader);

    }

    glfwSwapBuffers(window);
    glfwPollEvents();        

    // GLubyte * pixels = new GLubyte[3*screenWidth*screenHeight];
    // glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    // std::string image_title = "image/img"+std::to_string(idx)+".png";
    // stbi_flip_vertically_on_write(true);
    // stbi_write_png(image_title.c_str(), screenWidth, screenHeight, 3, pixels, screenWidth*3);
    // delete[] pixels;

}

void OpenglPointProcessing::draw_point_global(std::vector<PointCloud> & g_pt_cld, GLfloat size){


    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    while(!glfwWindowShouldClose(window)){
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 projection = glm::mat4(1.0f);

        point_shader.use();
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();



        // glm::mat4 Ortho_T = glm::mat4(1.0);
        // glm::translate(Ortho_T, glm::vec3(0.0, 0.0, 0.5));
        // glm::mat4 Ortho_S = glm::mat4(1.0);
        // glm::translate(Ortho_S, glm::vec3(1.0, 1.0, 0.0001));
        // glm::mat4 Ortho_V = Ortho_S*Ortho_T;
        // projection = Ortho_V;


        for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

            int data_size = g_pt_cld[data_id].point_cloud.cols();

            float vertices[6*data_size];
        
            for(int i = 0; i<data_size; ++i){
                for(int j = 0; j<3; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i);
                }
                for(int j = 3; j<6; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
                }
            }

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);


            //Model and camera;


            model = eigen_mat4_to_glm_mat4(g_pt_cld[data_id].state_matrix);
            projection = glm::perspective(glm::radians(camera->Zoom),
                                            float(screenWidth)/float(screenHeight), 
                                            0.1f, 
                                            100.0f);


            point_shader.setMat4("model", model);
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);
            point_shader.setBool("use_texture_in", false);
            
            glBindVertexArray(VAO);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, data_size);
            glBindVertexArray(0);

        }

        std::cout<<"Position"<<std::endl;
        std::cout<<camera->Position.x<<", "<<camera->Position.y<<", "<<camera->Position.z<<std::endl;

        std::cout<<"Front"<<std::endl;
        std::cout<<camera->Front.x<<", "<<camera->Front.y<<", "<<camera->Front.z<<std::endl;

        std::cout<<"Up"<<std::endl;
        std::cout<<camera->Up.x<<", "<<camera->Up.y<<", "<<camera->Up.z<<std::endl;


        glfwSwapBuffers(window);
        glfwPollEvents();        
    }
}

void OpenglPointProcessing::terminate(){
    glfwTerminate();
}

void OpenglPointProcessing::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}


void OpenglPointProcessing::draw_axis(float line_length, float line_width, Shader * shader){
    float vertices[]{
        0.0f, 0.0f, 0.0f,                   1.0f, 0.0f, 0.0f,//v0
        line_length*1.0f, 0.0f, 0.0f,       1.0f, 0.0f, 0.0f,//vx
        0.0f, 0.0f, 0.0f,                   0.0f, 1.0f, 0.0f,//v0
        0.0f, line_length*1.0f, 0.0f,       0.0f, 1.0f, 0.0f,//vy
        0.0f, 0.0f, 0.0f,                   0.0f, 0.0f, 1.0f,//v0
        0.0f, 0.0f, line_length*1.0f,       0.0f, 0.0f, 1.0f//vz
    };

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = glm::mat4(1.0f);

    shader->setMat4("model", model);

    glLineWidth(line_width);
    // glDrawElements(GL_LINES, 6, GL_UNSIGNED_BYTE, 0);
    glDrawArrays(GL_LINES, 0, 6);
    
}

void OpenglPointProcessing::processInput_end(){
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        camera->MovementSpeed = camera->FastSpeed;
    } else {
        camera->MovementSpeed = camera->OriginalSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->ProcessKeyboard(FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        camera->ProcessKeyboard(BACKWARD, deltaTime);
    }    
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->ProcessKeyboard(LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->ProcessKeyboard(RIGHT, deltaTime);
    }
    if(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){

    }


    if(glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS){
        camera->Position = glm::vec3(-1.67919, 5.02597, 0.0632081);
        camera->Front = glm::vec3(0.393309, -0.895984, 0.206204);
        camera->Up = glm::vec3(0.0828833, -0.188813, -0.978509);
    }
    if(glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS){
        camera->Position = glm::vec3(3.53543, 4.85189, -0.181782);
        camera->Front = glm::vec3(-0.325681, -0.89968, 0.290702);
        camera->Up = glm::vec3(-0.0989494, -0.273344, -0.956814);
    }
    if(glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS){
        camera->Position = glm::vec3(3.55824, -20.3131, 1.08095);
        camera->Front = glm::vec3(-0.50642,0.859736 ,0.066274);
        camera->Up = glm::vec3(-0.0336364, 0.0571037, -0.997802);
    }
    if(glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS){
        camera->Position = glm::vec3(-2.39743, -22.9813, -2.20565);
        camera->Front = glm::vec3(0.215845, 0.898998, 0.38107);
        camera->Up = glm::vec3(0.0889647, 0.37054, -0.924546);
    }

    if(glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS){
        camera->Position = glm::vec3(0.478709, -7.34962, -14.2159);
        camera->Front = glm::vec3(-0.0174507, 0.000243689, 0.999848);
        camera->Up = glm::vec3(-0.99975, 0.013961, -0.0174524);
    }

    if(glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS){
        camera->Position = glm::vec3(-13.5037, -7.05344, -0.170279);
        camera->Front = glm::vec3(0.991178, -0.00864102, 0.132256);
        camera->Up = glm::vec3(0.132251, -0.00115296, -0.991216);
    }
    if(glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS){
        camera->Position = glm::vec3(0.418237, 3.29749, 1.0539);
        camera->Front = glm::vec3(0.0608768, -0.995325, 0.0749787);
        camera->Up = glm::vec3(0.00457735, -0.0748389, -0.997185);
    }
}

void OpenglPointProcessing::mouse_callback(GLFWwindow * window, double xpos, double ypos) {

    OpenglPointProcessing *ogl_pointer =
         static_cast<OpenglPointProcessing*>(glfwGetWindowUserPointer(window));
    ogl_pointer->mouse_callback_function(xpos, ypos);

}

void OpenglPointProcessing::scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
    OpenglPointProcessing *ogl_pointer =
         static_cast<OpenglPointProcessing*>(glfwGetWindowUserPointer(window));
    ogl_pointer->scroll_callback_function(xoffset, yoffset);
}

void OpenglPointProcessing::mouse_callback_function(double xpos, double ypos) {
    if(firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos-lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    camera->ProcessMouseMovement(xoffset, yoffset);
}

void OpenglPointProcessing::scroll_callback_function(double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}


void OpenglPointProcessing::insertImages(cv::Mat & img){
    cv::Mat img_temp;
    cv::cvtColor(img, img_temp, cv::COLOR_BGR2RGB);
    cv::flip(img_temp, img_temp, -1);
    imgs.push_back(img_temp);
}


void OpenglPointProcessing::draw_plane_global(gtsam::Values & results){

    // glfwSetCursorPosCallback(window, mouse_callback);
    // glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // std::vector<Eigen::Vector3f> loc_0, loc_1, loc_2, loc_3;
    // loc_0.resize(results.size());
    // loc_1.resize(results.size());
    // loc_2.resize(results.size());
    // loc_3.resize(results.size());
    // std::vector<gtsamexample::StatePlane> state;
    // state.resize(results.size());

    // for(int data_id = 0; data_id < results.size(); ++data_id){
    //     state[data_id] = results.at<gtsamexample::StatePlane>(data_id);

    //     Eigen::Vector3f line_0(focal_length, -img_center_x, -img_center_y);
    //     Eigen::Vector3f line_1(focal_length, img_center_x, -img_center_y);
    //     Eigen::Vector3f line_2(focal_length, -img_center_x, img_center_y);
    //     Eigen::Vector3f line_3(focal_length, img_center_x, img_center_y);
    //     Eigen::Vector4f plane_param = Eigen::Vector4f(state[data_id].nx, state[data_id].ny, state[data_id].nz, state[data_id].d);

    //     loc_0[data_id] = line_0 * (-state[data_id].d/(line_0.transpose()*plane_param.segment(0,3)));
    //     loc_1[data_id] = line_1 * (-state[data_id].d/(line_1.transpose()*plane_param.segment(0,3)));
    //     loc_2[data_id] = line_2 * (-state[data_id].d/(line_2.transpose()*plane_param.segment(0,3)));
    //     loc_3[data_id] = line_3 * (-state[data_id].d/(line_3.transpose()*plane_param.segment(0,3)));
    // }

    // plane_shader.use();

    // while(!glfwWindowShouldClose(window)){
    //     processInput_end();
    //     glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    //     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

    //     glm::mat4 model = glm::mat4(1.0f);
    //     glm::mat4 view = glm::mat4(1.0f);
    //     glm::mat4 projection = glm::mat4(1.0f);

    //     projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
    //     view = camera->GetViewMatrix();

    //     for(int data_id = 0; data_id < results.size(); ++data_id){

    //         int height = imgs[data_id].rows;
    //         int width = imgs[data_id].cols;

    //         float vertices[] = {
    //             //Position                                                          //TexCoord
    //             loc_0[data_id](0),   loc_0[data_id](1),   loc_0[data_id](2),       0.0f,   1.0f,
    //             loc_1[data_id](0),   loc_1[data_id](1),   loc_1[data_id](2),       1.0f,   1.0f,
    //             loc_2[data_id](0),   loc_2[data_id](1),   loc_2[data_id](2),       0.0f,   0.0f,
    //             loc_3[data_id](0),   loc_3[data_id](1),   loc_3[data_id](2),       1.0f,   0.0f
    //         };

    //         unsigned int indices[] = {
    //             0,  1,  2,
    //             1,  2,  3
    //         };

    //         glBindVertexArray(VAO);
    //         glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //         glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    //         glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    //         glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices, GL_STATIC_DRAW);

    //         glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)0);
    //         glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(float), (void*)(3*sizeof(float)));
    //         glEnableVertexAttribArray(0);
    //         glEnableVertexAttribArray(1);

    //         unsigned int texture;
    //         glGenTextures(1, &texture);
    //         glBindTexture(GL_TEXTURE_2D, texture);

    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    //         if(imgs[data_id].data){
    //             glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, imgs[data_id].data);
    //         }

    //         //Model and camera;
    //         Eigen::Matrix4f cur_state;
            
    //         c_trans.xyzrpy2t(state[data_id].x, state[data_id].y, state[data_id].z, state[data_id].roll, state[data_id].pitch, state[data_id].yaw , &cur_state);

    //         model = eigen_mat4_to_glm_mat4(cur_state);

    //         projection = glm::perspective(glm::radians(camera->Zoom),
    //                                         float(screenWidth)/float(screenHeight), 
    //                                         0.1f, 
    //                                         100.0f);

    //         plane_shader.setMat4("model", model);
    //         plane_shader.setMat4("view", view);
    //         plane_shader.setMat4("projection", projection);


    //         glBindVertexArray(VAO);
    //         glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    //     }

    //     glfwSwapBuffers(window);
    //     glfwPollEvents();        
    // }
}


void OpenglPointProcessing::draw_plane_global_wo_texture(gtsam::Values & results){

    // glfwSetCursorPosCallback(window, mouse_callback);
    // glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // std::vector<Eigen::Vector3f> loc_0, loc_1, loc_2, loc_3;
    // loc_0.resize(results.size());
    // loc_1.resize(results.size());
    // loc_2.resize(results.size());
    // loc_3.resize(results.size());
    // std::vector<gtsamexample::StatePlane> state;
    // state.resize(results.size());

    // for(int data_id = 0; data_id < results.size(); ++data_id){
    //     state[data_id] = results.at<gtsamexample::StatePlane>(data_id);

    //     Eigen::Vector3f line_0(focal_length, -img_center_x, -img_center_y);
    //     Eigen::Vector3f line_1(focal_length, img_center_x, -img_center_y);
    //     Eigen::Vector3f line_2(focal_length, -img_center_x, img_center_y);
    //     Eigen::Vector3f line_3(focal_length, img_center_x, img_center_y);
    //     Eigen::Vector4f plane_param = Eigen::Vector4f(state[data_id].nx, state[data_id].ny, state[data_id].nz, state[data_id].d);

    //     loc_0[data_id] = line_0 * (-state[data_id].d/(line_0.transpose()*plane_param.segment(0,2)));
    //     loc_1[data_id] = line_1 * (-state[data_id].d/(line_1.transpose()*plane_param.segment(0,2)));
    //     loc_2[data_id] = line_2 * (-state[data_id].d/(line_2.transpose()*plane_param.segment(0,2)));
    //     loc_3[data_id] = line_3 * (-state[data_id].d/(line_3.transpose()*plane_param.segment(0,2)));
    // }

    // point_shader.use();

    // while(!glfwWindowShouldClose(window)){
    //     processInput_end();
    //     glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    //     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

    //     glm::mat4 model = glm::mat4(1.0f);
    //     glm::mat4 view = glm::mat4(1.0f);
    //     glm::mat4 projection = glm::mat4(1.0f);

    //     projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
    //     view = camera->GetViewMatrix();

    //     for(int data_id = 0; data_id < results.size(); ++data_id){

    //         int height = imgs[data_id].rows;
    //         int width = imgs[data_id].cols;

    //         float vertices[] = {
    //             //Position                                                          //Colors
    //             loc_0[data_id](0),   loc_0[data_id](1),   loc_0[data_id](2),       0.5f,   0.5f,    0.5f,
    //             loc_1[data_id](0),   loc_1[data_id](1),   loc_1[data_id](2),       0.5f,   0.5f,    0.5f,
    //             loc_2[data_id](0),   loc_2[data_id](1),   loc_2[data_id](2),       0.5f,   0.5f,    0.5f,
    //             loc_3[data_id](0),   loc_3[data_id](1),   loc_3[data_id](2),       0.5f,   0.5f,    0.5f
    //         };

    //         unsigned int indices[] = {
    //             0,  1,  2,
    //             1,  2,  3
    //         };

    //         glBindVertexArray(VAO);
    //         glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //         glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    //         glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    //         glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices, GL_STATIC_DRAW);

    //         glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    //         glEnableVertexAttribArray(0);

    //         glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    //         glEnableVertexAttribArray(1);

    //         //Model and camera;
    //         Eigen::Matrix4f cur_state;
            
    //         c_trans.xyzrpy2t(state[data_id].x, state[data_id].y, state[data_id].z, state[data_id].roll, state[data_id].pitch, state[data_id].yaw , &cur_state);

    //         model = eigen_mat4_to_glm_mat4(cur_state);

    //         point_shader.setMat4("model", model);
    //         point_shader.setMat4("view", view);
    //         point_shader.setMat4("projection", projection);

    //         glBindVertexArray(VAO);
    //         glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    //     }

    //     glfwSwapBuffers(window);
    //     glfwPollEvents();        
    // }
}


void OpenglPointProcessing::draw_surfels(gtsam::Values & results){

    // glfwSetCursorPosCallback(window, mouse_callback);
    // glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // std::vector<glm::mat4> circle_transformation;
    // circle_transformation.resize(results.size());


    // std::vector<gtsamexample::StatePlane> state;
    // state.resize(results.size());

    // for(int data_id = 0; data_id < results.size(); ++data_id){
    //     gtsamexample::StatePlane state_tmp = results.at<gtsamexample::StatePlane>(data_id);
    //     state[data_id] = state_tmp;
    //     Eigen::Vector3f a{-1.0, 0.0, 0.0};
    //     Eigen::Vector3f b{(float)state_tmp.nx, (float)state_tmp.ny, (float)state_tmp.nz};
    //     Eigen::Vector3f v = skew_symmetric(a)*b;


    //     float s2 = v.transpose()*v;
    //     Eigen::Matrix3f R;

    //     if(s2==0){
    //         R = Eigen::Matrix3f::Identity(3,3);
            
    //     } else{
    //         float s = sqrt(v.transpose()*v);
    //         float c = a.transpose()*b;
    //         R = Eigen::Matrix3f::Identity(3,3) + skew_symmetric(v) +
    //                             skew_symmetric(v)*skew_symmetric(v)*(1-c)/(s*s);
    //     }


    //     Eigen::Vector3f p{-(float)state_tmp.d/((float)state_tmp.nx), 0.0, 0.0};

    //     Eigen::Matrix4f T_mat;
    //     T_mat<<R,p,0,0,0,1;

    //     circle_transformation[data_id] = eigen_mat4_to_glm_mat4(T_mat);
    // }

    // int sides = 150;

    // float vertices[6*(sides+2)];
    // float r = 0.2f;

    // for(int i = 0; i<(sides+2); ++i){
    //     if(i==0){
    //         vertices[0] = 0.0;
    //         vertices[1] = 0.0;
    //         vertices[2] = 0.0;
    //         vertices[3] = 1.0f;
    //         vertices[4] = 1.0f;
    //         vertices[5] = 0.0f;
    //         continue;
    //     }
    //     vertices[i*6+0] = 0.0;
    //     vertices[i*6+1] = r*cos((360.0f/float(sides))*(i-1)*M_PI/180.0f);
    //     vertices[i*6+2] = r*sin((360.0f/float(sides))*(i-1)*M_PI/180.0f);

    //     vertices[i*6+3] = 1.0f;
    //     vertices[i*6+4] = 1.0f;
    //     vertices[i*6+5] = 0.0f;
    // }

    // point_shader.use();

    // while(!glfwWindowShouldClose(window)){
    //     processInput_end();
    //     glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    //     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  

    //     glm::mat4 model = glm::mat4(1.0f);
    //     glm::mat4 view = glm::mat4(1.0f);
    //     glm::mat4 projection = glm::mat4(1.0f);

    //     projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
    //     view = camera->GetViewMatrix();

    //     for(int data_id = 0; data_id < results.size(); ++data_id){

    //         int height = imgs[data_id].rows;
    //         int width = imgs[data_id].cols;


    //         glBindVertexArray(VAO);
    //         glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //         glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    //         glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    //         glEnableVertexAttribArray(0);

    //         glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    //         glEnableVertexAttribArray(1);

    //         //Model and camera;
    //         Eigen::Matrix4f cur_state;
            
    //         c_trans.xyzrpy2t(state[data_id].x, 
    //                          state[data_id].y, 
    //                          state[data_id].z, 
    //                          state[data_id].roll, state[data_id].pitch, state[data_id].yaw , &cur_state);

    //         model = eigen_mat4_to_glm_mat4(cur_state) * circle_transformation[data_id];
    //         // model = eigen_mat4_to_glm_mat4(cur_state);

    //         point_shader.setMat4("model", model);
    //         point_shader.setMat4("view", view);
    //         point_shader.setMat4("projection", projection);

    //         glDrawArrays(GL_TRIANGLE_FAN, 0, sides+2);
    //         draw_axis(2.0f, 20.0f, &point_shader);

    //     }

    //     glfwSwapBuffers(window);
    //     glfwPollEvents();        
    // }
}


Eigen::Matrix3f OpenglPointProcessing::skew_symmetric(Eigen::Vector3f& vector){
    Eigen::Matrix3f result;
    result(0, 0) = 0.0;
    result(0, 1) = -vector(2);
    result(0, 2) = vector(1);
    result(1, 0) = vector(2);
    result(1, 1) = 0;
    result(1, 2) = -vector(0);
    result(2, 0) = -vector(1);
    result(2, 1) = vector(0);
    result(2, 2) = 0;
    return result;
}


void OpenglPointProcessing::draw_point_global_double_window_test(std::vector<PointCloud> & g_pt_cld, GLfloat size){

    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    while(!glfwWindowShouldClose(window)){
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  



        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);
        glm::mat4 projection = glm::mat4(1.0f);

        glViewport(0, 0, screenWidth/2, screenHeight);
        point_shader.use();
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/2.0f/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();

        for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

            int data_size = g_pt_cld[data_id].point_cloud.cols();

            float vertices[6*data_size];
        
            for(int i = 0; i<data_size; ++i){
                for(int j = 0; j<3; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i);
                }
                for(int j = 3; j<6; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
                }
            }

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);


            //Model and camera;


            model = eigen_mat4_to_glm_mat4(g_pt_cld[data_id].state_matrix);

            point_shader.setMat4("model", model);
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);
            point_shader.setBool("use_texture_in", false);
            
            glBindVertexArray(VAO);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, data_size);
            glBindVertexArray(0);

        }

        glViewport(screenWidth/2, 0, screenWidth/2, screenHeight);
        point_shader.use();
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/2.0f/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();

        for(int data_id = 0; data_id<g_pt_cld.size(); ++data_id){

            int data_size = g_pt_cld[data_id].point_cloud.cols();

            float vertices[6*data_size];
        
            for(int i = 0; i<data_size; ++i){
                for(int j = 0; j<3; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_cloud(j,i);
                }
                for(int j = 3; j<6; ++j){
                    vertices[i*6+j] = g_pt_cld[data_id].point_color(j-3,i)/255.0f;
                }
            }

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);


            //Model and camera;


            model = eigen_mat4_to_glm_mat4(g_pt_cld[data_id].state_matrix);

            point_shader.setMat4("model", model);
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);
            point_shader.setBool("use_texture_in", false);
            
            glBindVertexArray(VAO);
            glPointSize(3.0);
            glDrawArrays(GL_POINTS, 0, data_size);
            glBindVertexArray(0);

        }


        glfwSwapBuffers(window);
        glfwPollEvents();        
    }
}
