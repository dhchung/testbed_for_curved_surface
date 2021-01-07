#include "opengl_rendering.h"


OpenglRendering::OpenglRendering(std::string window_name){
    w_name = window_name;
    screenWidth = 1600;
    screenHeight = 900;

    camera = new Camera(glm::vec3(-2.0f, 0.0f, -1.0f), glm::vec3(0.0f,0.0f,-1.0f));

    lastX = (float)screenWidth/2.0f;
    lastY = (float)screenHeight/2.0f;
    firstMouse = true;

    deltaTime = 0.0f;
    lastFrame = 0.0f;
    

    imgs.clear();
}

OpenglRendering::~OpenglRendering(){

}

void OpenglRendering::init_opengl(){
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

    surfel_shader.InitShader("shaders/surfel/vertex_shader.vs", "shaders/surfel/fragment_shader.fs");

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenVertexArrays(1, &VAO);
    glEnable(GL_DEPTH_TEST);  

}

void OpenglRendering::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglRendering::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

glm::mat4 OpenglRendering::eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4){

    glm::mat4 result;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            result[j][i] = e_mat4(i,j);
        }
    }
    return result;
}

void OpenglRendering::terminate(){
    glfwTerminate();
}

void OpenglRendering::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}


void OpenglRendering::draw_axis(float line_length, float line_width, Shader * shader){
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

void OpenglRendering::draw_camera(Params & params, glm::mat4 &model_T, float line_length, float line_width, std::vector<float> &color, Shader * shader){

    float x = line_length;
    float y = params.camParam.img_center_x/params.camParam.focal_length*line_length;
    float z = params.camParam.img_center_y/params.camParam.focal_length*line_length;

    float vertices[]{
        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, y, -z,                           color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, y, z,                            color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, -y, -z,                          color[0], color[1], color[2],

        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        x, -y, z,                           color[0], color[1], color[2],

        x, y, -z,                           color[0], color[1], color[2],//v0
        x, -y, -z,                          color[0], color[1], color[2],

        x, -y, -z,                          color[0], color[1], color[2],
        x, -y, z,                           color[0], color[1], color[2],

        x, -y, z,                           color[0], color[1], color[2],
        x, y, z,                            color[0], color[1], color[2],

        x, y, z,                            color[0], color[1], color[2],
        x, y, -z,                           color[0], color[1], color[2]
    };
    

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = model_T;

    shader->setMat4("model", model);

    glLineWidth(line_width);
    glDrawArrays(GL_LINES, 0, 16);
}


void OpenglRendering::draw_arrow(glm::mat4 &model_T, float line_length, float arrow_length, float line_width, std::vector<float> &color, Shader * shader){

    float arrow_sqrt = arrow_length / sqrt(2.0f);
    float arrow_short = line_length - arrow_sqrt;

    float vertices[]{
        0.0f, 0.0f, 0.0f,                   color[0], color[1], color[2],//v0
        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],

        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],//v0
        arrow_short, arrow_sqrt, 0,         color[0], color[1], color[2],

        line_length, 0.0f, 0.0f,            color[0], color[1], color[2],//v0
        arrow_short, -arrow_sqrt, 0,        color[0], color[1], color[2]
    };
    

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = model_T;

    shader->setMat4("model", model);

    glLineWidth(line_width);
    // glDrawElements(GL_LINES, 6, GL_UNSIGNED_BYTE, 0);
    glDrawArrays(GL_LINES, 0, 6);
}

void OpenglRendering::processInput_end(){
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

}

void OpenglRendering::mouse_callback(GLFWwindow * window, double xpos, double ypos) {

    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->mouse_callback_function(xpos, ypos);

}

void OpenglRendering::scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->scroll_callback_function(xoffset, yoffset);
}

void OpenglRendering::mouse_callback_function(double xpos, double ypos) {
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

void OpenglRendering::scroll_callback_function(double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}


Eigen::Matrix3f OpenglRendering::skew_symmetric(Eigen::Vector3f& vector){
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


void OpenglRendering::draw_surfels(std::vector<std::vector<float>>& surfels,  std::vector<float> & color){
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    std::vector<glm::mat4> circle_transformation;
    circle_transformation.resize(surfels.size());
    std::vector<glm::vec3> normals;
    normals.resize(surfels.size());

    for(int i=0; i<surfels.size(); ++i) {
        Eigen::Vector3f a{1.0f, 0.0f, 0.0f};
        Eigen::Vector3f b{surfels[i][0], surfels[i][1], surfels[i][2]};
        Eigen::Vector3f v = skew_symmetric(a)*b;

        float s2 = v.transpose()*v;
        Eigen::Matrix3f R;

        if(s2==0) {
            Eigen::Vector3f diff = a-b;
            if(diff.transpose()*diff < 0.1){
                R = Eigen::Matrix3f::Identity(3,3);
            }else{
                R = Eigen::Matrix3f::Identity(3,3);
                R(0,0) = -1.0f;
                R(1,1) = -1.0f;
            }
        } else {
            float s = sqrt(v.transpose()*v);
            float c = a.transpose()*b;
            R = Eigen::Matrix3f::Identity(3,3) + 
                skew_symmetric(v) + 
                skew_symmetric(v)*skew_symmetric(v)*(1-c)/(s*s);
        }

        Eigen::Vector3f p{surfels[i][3], surfels[i][4], surfels[i][5]};
        Eigen::Matrix4f T_mat;
        T_mat<<R,p,0,0,0,1;
        circle_transformation[i] = eigen_mat4_to_glm_mat4(T_mat);

        normals[i][0] = surfels[i][0];
        normals[i][1] = surfels[i][1];
        normals[i][2] = surfels[i][2];
    }

    int sides = 150;
    float vertices[3*(sides+2)];
    float r = 0.2f;
    float edge_width = 0.02;

    float vertices_edge_strip[6*(sides+2)];

    for(int i = 0; i <(sides+2); ++i) {
        if(i==0){
            vertices[0] = 0.0f;
            vertices[1] = 0.0f;
            vertices[2] = 0.0f;
            continue;
        }
        vertices[i*3+0] = 0.0f;
        vertices[i*3+1] = r*cos(2.0f/float(sides)*(i-1)*M_PI);
        vertices[i*3+2] = r*sin(2.0f/float(sides)*(i-1)*M_PI);
    }

    for(int i = 0; i < sides+2; ++i) {
        vertices_edge_strip[6*i+0] = 0.0f;
        vertices_edge_strip[6*i+1] = r*cos(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+2] = r*sin(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+3] = 0.0f;
        vertices_edge_strip[6*i+4] = (r+edge_width)*cos(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+5] = (r+edge_width)*sin(2.0f/float(sides)*i*M_PI);

    }



    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);


    surfel_shader.use();

    glm::vec3 lightDir(-1.0f, 0.0f, 1.0f);
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
    glm::vec3 objectColor(color[0], color[1], color[2]);

    glm::vec3 edgeColor(0.5f, 0.5f, 0.5f);

    surfel_shader.setVec3("lightDir", lightDir);
    surfel_shader.setVec3("lightColor", lightColor);
    surfel_shader.setVec3("objectColor", objectColor);


    while(!glfwWindowShouldClose(window)){
        surfel_shader.use();
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();

        for(int i = 0; i<surfels.size(); ++i) {
            surfel_shader.setVec3("objectColor", objectColor);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);


            //Model and camera;
            Eigen::Matrix4f cur_state;
            
            model = circle_transformation[i];

            surfel_shader.setMat4("model", model);
            surfel_shader.setMat4("view", view);
            surfel_shader.setMat4("projection", projection);

            surfel_shader.setVec3("objectNormal", normals[i]);

            glDrawArrays(GL_TRIANGLE_FAN, 0, sides+2);

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_edge_strip), vertices_edge_strip, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            surfel_shader.setVec3("objectColor", edgeColor);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, sides*2);

        }

        point_shader.use();
        point_shader.setMat4("view", view);
        point_shader.setMat4("projection", projection);

        draw_axis(1.0f, 20.0f, &point_shader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}


void OpenglRendering::draw_surfels_init_n_final(std::vector<Eigen::Matrix4f> &state0,
                                                std::vector<Eigen::Matrix4f> &state1,    
                                                std::vector<std::vector<float>>& surfels0,
                                                std::vector<std::vector<float>>& surfels1,
                                                std::vector<float> & color0,
                                                std::vector<float> & color1,
                                                Params & params){
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    std::vector<glm::mat4> circle_transformation0;
    circle_transformation0.resize(surfels0.size());

    std::vector<glm::mat4> circle_transformation1;
    circle_transformation1.resize(surfels1.size());


    std::vector<glm::vec3> normals0;
    normals0.resize(surfels0.size());

    std::vector<glm::vec3> normals1;
    normals1.resize(surfels1.size());

    for(int i=0; i<surfels0.size(); ++i) {
        Eigen::Vector3f a{1.0f, 0.0f, 0.0f};
        Eigen::Vector3f b{surfels0[i][0], surfels0[i][1], surfels0[i][2]};
        b = b/(sqrt(b.transpose()*b));
        Eigen::Vector3f v = skew_symmetric(a)*b;

        float s2 = v.transpose()*v;
        Eigen::Matrix3f R;

        if(s2==0) {
            Eigen::Vector3f diff = a-b;
            if(diff.transpose()*diff < 0.1){
                R = Eigen::Matrix3f::Identity(3,3);
            }else{
                R = Eigen::Matrix3f::Identity(3,3);
                R(0,0) = -1.0f;
                R(1,1) = -1.0f;
            }
        } else {
            float s = sqrt(v.transpose()*v);
            float c = a.transpose()*b;
            R = Eigen::Matrix3f::Identity(3,3) + 
                skew_symmetric(v) + 
                skew_symmetric(v)*skew_symmetric(v)*(1-c)/(s*s);
        }

        Eigen::Vector3f p{surfels0[i][3], surfels0[i][4], surfels0[i][5]};
        Eigen::Matrix4f T_mat;
        T_mat<<R,p,0,0,0,1;
        circle_transformation0[i] = eigen_mat4_to_glm_mat4(T_mat);

        normals0[i][0] = surfels0[i][0];
        normals0[i][1] = surfels0[i][1];
        normals0[i][2] = surfels0[i][2];

    }

    for(int i=0; i<surfels1.size(); ++i) {
        Eigen::Vector3f a{1.0f, 0.0f, 0.0f};
        Eigen::Vector3f b{surfels1[i][0], surfels1[i][1], surfels1[i][2]};
        Eigen::Vector3f v = skew_symmetric(a)*b;

        float s2 = v.transpose()*v;
        Eigen::Matrix3f R;

        if(s2==0) {
            Eigen::Vector3f diff = a-b;
            if(diff.transpose()*diff < 0.1){
                R = Eigen::Matrix3f::Identity(3,3);
            }else{
                R = Eigen::Matrix3f::Identity(3,3);
                R(0,0) = -1.0f;
                R(1,1) = -1.0f;
            }
        } else {
            float s = sqrt(v.transpose()*v);
            float c = a.transpose()*b;
            R = Eigen::Matrix3f::Identity(3,3) + 
                skew_symmetric(v) + 
                skew_symmetric(v)*skew_symmetric(v)*(1-c)/(s*s);
        }

        Eigen::Vector3f p{surfels1[i][3], surfels1[i][4], surfels1[i][5]};
        Eigen::Matrix4f T_mat;
        T_mat<<R,p,0,0,0,1;
        circle_transformation1[i] = eigen_mat4_to_glm_mat4(T_mat);

        normals1[i][0] = surfels1[i][0];
        normals1[i][1] = surfels1[i][1];
        normals1[i][2] = surfels1[i][2];
    }

    int sides = 150;
    float vertices[3*(sides+2)];
    float r = 0.2f;
    float edge_width = 0.02;

    float vertices_edge_strip[6*(sides+2)];

    for(int i = 0; i <(sides+2); ++i) {
        if(i==0){
            vertices[0] = 0.0f;
            vertices[1] = 0.0f;
            vertices[2] = 0.0f;
            continue;
        }
        vertices[i*3+0] = 0.0f;
        vertices[i*3+1] = r*cos(2.0f/float(sides)*(i-1)*M_PI);
        vertices[i*3+2] = r*sin(2.0f/float(sides)*(i-1)*M_PI);
    }

    for(int i = 0; i < sides+2; ++i) {
        vertices_edge_strip[6*i+0] = 0.0f;
        vertices_edge_strip[6*i+1] = r*cos(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+2] = r*sin(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+3] = 0.0f;
        vertices_edge_strip[6*i+4] = (r+edge_width)*cos(2.0f/float(sides)*i*M_PI);
        vertices_edge_strip[6*i+5] = (r+edge_width)*sin(2.0f/float(sides)*i*M_PI);

    }



    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);


    surfel_shader.use();

    glm::vec3 lightDir(-1.0f, 0.0f, 1.0f);
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f);

    glm::vec3 objectColorGray(0.4, 0.4, 0.4);

    glm::vec3 objectColor0(color0[0], color0[1], color0[2]);
    glm::vec3 objectColor1(color1[0], color1[1], color1[2]);

    glm::vec3 edgeColor(0.5f, 0.5f, 0.5f);

    surfel_shader.setVec3("lightDir", lightDir);
    surfel_shader.setVec3("lightColor", lightColor);


    std::vector<float> cam_color0{1.0, 0.0, 0.0};
    std::vector<float> cam_color1{0.0, 1.0, 0.0};


    while(!glfwWindowShouldClose(window)){
        surfel_shader.use();
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();

        for(int i = 0; i<surfels0.size(); ++i) {
            surfel_shader.use();
            if(i==0) {
                surfel_shader.setVec3("objectColor", objectColorGray);
            } else {
                surfel_shader.setVec3("objectColor", objectColor0);
            }
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);


            model = circle_transformation0[i];

            surfel_shader.setMat4("model", model);
            surfel_shader.setMat4("view", view);
            surfel_shader.setMat4("projection", projection);

            surfel_shader.setVec3("objectNormal", normals0[i]);

            glDrawArrays(GL_TRIANGLE_FAN, 0, sides+2);

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_edge_strip), vertices_edge_strip, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            surfel_shader.setVec3("objectColor", edgeColor);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, sides*2);


            glm::mat4 state = eigen_mat4_to_glm_mat4(state0[i]);
            // draw_axis(1.0f, 20.0f, &point_shader);

            point_shader.use();
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);
            draw_camera(params, state, 0.2f, 5.0f, cam_color0, &point_shader);
            draw_arrow(model, 0.5f, 0.2f, 5.0f, cam_color0, &point_shader);
        }


        for(int i = 0; i<surfels1.size(); ++i) {
            surfel_shader.use();
            surfel_shader.setVec3("objectColor", objectColor1);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glm::mat4 scale = glm::mat4(1.0f);
            scale[0][0] = 2.0f;
            scale[1][1] = 2.0f;
            scale[2][2] = 2.0f;


            model = circle_transformation1[i]*scale;
            // model = circle_transformation1[i];
            surfel_shader.setMat4("model", model);
            surfel_shader.setMat4("view", view);
            surfel_shader.setMat4("projection", projection);

            surfel_shader.setVec3("objectNormal", normals1[i]);

            glDrawArrays(GL_TRIANGLE_FAN, 0, sides+2);

            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_edge_strip), vertices_edge_strip, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            surfel_shader.setVec3("objectColor", edgeColor);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, sides*2);

            glm::mat4 state = eigen_mat4_to_glm_mat4(state1[i]);
            // draw_axis(1.0f, 20.0f, &point_shader);

            point_shader.use();
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);
            draw_camera(params, state, 0.2f, 5.0f, cam_color1, &point_shader);
            model = glm::scale(model, glm::vec3(0.5, 0.5, 0.5));
            draw_arrow(model, 0.5f, 0.2f, 5.0f, cam_color1, &point_shader);


        }




        point_shader.use();
        point_shader.setMat4("view", view);
        point_shader.setMat4("projection", projection);

        draw_axis(1.0f, 20.0f, &point_shader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}