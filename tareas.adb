with Ada.Real_Time; use Ada.Real_Time;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;
with Robot_Ackermann; use Robot_Ackermann;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;

procedure tareas is
   ---Recurso compartido Pantalla-----
   protected Pantalla is
      entry Entrar(Frase : Unbounded_String);
      procedure Salir;
   private
      ---Valor : Integer := 0;
      Libre : Boolean :=True;
   end Pantalla;

   protected body Pantalla is
      
      entry Entrar(Frase : Unbounded_String) when Libre is
      begin
         Libre := False;
          --Mostramos por pantalla--
         Put_Line("--- Datos del Robot ---");
         Put_Line (To_String(Frase));
      end Entrar;

      procedure Salir is
      begin
         Libre := True;
      end Salir;

   end Pantalla;

   ---Recurso compartido almacenamiento---
   protected Almacenamiento is
      procedure crear;
      entry Escribir(Frase2 : Unbounded_String);
      procedure Salir;
   private
      ---Valor : Integer := 0;
      Libre : Boolean :=True;
   end Almacenamiento;

   protected body Almacenamiento is
      procedure crear is
      Log_file : File_Type;
      begin
      Create (Log_file, Out_File, "robot.txt");
      Close(Log_file);
      end crear;


      entry Escribir (Frase2 : Unbounded_String) when Libre is
      Log_file : File_Type;
      begin
         Libre := False;
         Open(Log_file, Append_File,"robot.txt");
         Put_Line (Log_file,To_String(Frase2));
         Close(Log_file);
      end Escribir;

      procedure Salir is
      begin
         Libre := True;
      end Salir;

   end Almacenamiento;

   protected  Guardar is
      procedure crear;
      procedure GuardarM1(t : Time; t0 : Time; v_real : Float; e_x : Float; e_y : Float; pos_x : Float; pos_y : Float);
      procedure GuardarM2( psi_real : Float);
      procedure Escribir;
   private
      velocidad,error_x,error_y,posicion_x,posicion_y,psi_real,Angulo : Float;
      Tiempo,T_0 : Time;
   end Guardar;

   protected body Guardar is
      procedure crear is
      Log_file : File_Type;
      begin
      Create (Log_file, Out_File, "robot.txt");
      Close(Log_file);
      end crear;

      procedure GuardarM1(t : Time; t0 : Time; v_real : Float; e_x : Float; e_y : Float; pos_x : Float; pos_y : Float) is
      --  velocidad,error_x,error_y,posicion_x,posicion_y : Float;
      --  Tiempo,T_0 : Time;
      begin

      Tiempo := t;
      T_0 := t0;
      velocidad := v_real;
      error_x := e_x;
      error_y := e_y;
      posicion_x := pos_x;
      posicion_y := pos_y;
      end GuardarM1;

      procedure GuardarM2(psi_real : Float) is
      begin 
      Angulo := psi_real;
      end GuardarM2;


      procedure Escribir is
      Log_file : File_Type;
      --  velocidad,error_x,error_y,posicion_x,posicion_y,psi_real : Float;
      --  Tiempo,T_0 : Time;
      begin
         Open(Log_file, Append_File,"robot.txt");
         Put_Line (Log_file,Float'Image(Float(To_Duration(Tiempo-T_0))) & "," &Float'Image(Angulo * 180.0 / Ada.Numerics.Pi) &
        "," & Float'Image(velocidad) & "," & Float'Image(Sqrt(error_x**2 + error_y**2)) &
         "," & Float'Image(posicion_x) & "," & Float'Image(posicion_y));
         Close(Log_file);  
      end Escribir;

   end Guardar;

    ---Recurso compartido Tarjeta-----
   protected Tarjeta is
      entry Usar;
      procedure Salir;
   private
      Libre : Boolean :=True;
   end Tarjeta;

   protected body Tarjeta is
      
      entry Usar when Libre is
      begin
         Libre := False;
      end Usar;

      procedure Salir is
      begin
         Libre := True;
      end Salir;

   end Tarjeta;



------Definicion tareas
   task Control_M1;
   task Control_M2;

   ---Implementación de la tarea Control_M1--
   task body Control_M1 is

   tsim       : constant Positive := 200;
   T_muestreo : constant Float    := 0.5;
   t, t0, tfin  : Time;
   Ts : Time_Span := Milliseconds (Integer (T_muestreo * 1_000.0)); -- 0.5 s

   ----------------- Parametros de control -----------------
   rx, ry, pos_x, pos_y, ori_theta, e_x, e_y, v_ref, v_real, 
   K_nu, K_po, Long: Float;
   Ngrupo : Integer;
   CpC25 : constant Float := -0.4 / 24.0;
   type Target_X is array (1 .. 6) of Float;  -- Referencia en X
   type Target_Y is array (1 .. 6) of Float;  -- Referencia en Y

   ----------------- Definición de coordenadas ----------------- 
   Target_X_r : constant Target_X :=  (100.0, 300.0, 400.0, 300.0, 100.0, 0.0);
   Target_Y_r : constant Target_Y :=  (100.0, 150.0, 100.0, 150.0, 250.0, 0.0);

   ------------------ Variables de simulación ----------------- 
   Control_Robot : AgriCobIoT;
   M : Robot_State;

   ------------------Variable de escritura texto---------------
   Log_File : File_Type; -- Define Log_File como una variable de escritura
   c : Integer; --Contador error angulo  
   Frase : Unbounded_String; -- Variable para escritura 
   Frase2 : Unbounded_String; -- Variable para escritura 

   begin

         ------------- Inicialización de parametros ----------------- 
      -- IMPORTANTE: DEFINIR NUMERO DE GRUPO
      Ngrupo := 19; -- Grupo Nº 19
      Long := CpC25 * Float(Ngrupo - 1) + 0.8;
      M.L := Long; -- Longitud del robot
      M.Ts := T_muestreo; -- Tiempo de muestreo
      Initialize_Robot (Control_Robot, M); 

      ----------------- Variables temporales ----------------- 
      t    := Clock;
      t0   := Clock;
      tfin := t0 + Seconds (tsim);

      ----------------- Iniciación de variables ----------------- 
      rx         := 0.0;
      ry         := 0.0;
      pos_x      := 0.0;
      pos_y      := 0.0;
      ori_theta  := 0.0;
      e_x        := 0.0;
      e_y        := 0.0;
      v_ref      := 0.0;
      v_real     := 0.0;
      K_nu       := 1.0;  -- Ganancia para el control de velocidad
      K_po       := 5.0;  -- Ganancia para el control de ángulo
      c          := 0;    --Contado = 0

      Almacenamiento.crear;

   loop
      ----Usamos el recurso compartido de tarjeta----
      tarjeta.Usar;

      ----Calculo referencia------
          if t >= t0 + Seconds (5) then
         rx := Target_X_r(1);
         ry := Target_Y_r(1);
         if t >= t0 + Seconds (50) then
            rx := Target_X_r(2);
            ry := Target_Y_r(2);
         end if;
         if t >= t0 + Seconds (70) then
            rx := Target_X_r(3);
            ry := Target_Y_r(3);
         end if;
         if t >= t0 + Seconds (100) then
            rx := Target_X_r(4);
            ry := Target_Y_r(4);
         end if;
         if t >= t0 + Seconds (125) then
            rx := Target_X_r(5);
            ry := Target_Y_r(5);
         end if;
         if t >= t0 + Seconds (150) then
            rx := Target_X_r(6);
            ry := Target_Y_r(6);
         end if;
      end if;
          ---------------  LECTURA DE DATOS  -------------------------
      pos_x       := x (Control_Robot);
      pos_y       := y (Control_Robot);
      ori_theta   := theta (Control_Robot);

      delay 0.250;

            -----------------  REFERENCIA  -------------------------
      e_x     := rx - pos_x;
      e_y     := ry - pos_y;

            --------------  CONTROL TRACCIÓN  -------------------------
      -- Distancia al punto mediante cálculo por Pitágoras
      if abs(e_x) <= 0.001 and abs(e_y) <= 0.001 then
         v_ref := 0.0;
      else
         v_real := K_nu * Sqrt(e_x**2 + e_y**2);
      end if;

      -- Límite de velocidad
      v_real := Float'Min(v_real, 10.0);  -- Límite superior
      v_real := Float'Max(v_real, -10.0); -- Límite inferior

      delay 0.020;

       --------------  ESCRITURA DE DATOS  -------------------------
      v(Control_Robot, v_real);
      delay 0.010;
      
      --- Libreamos el recurso compartido de tarjeta----
      Tarjeta.Salir;

      -------Creamos String de salida-------
      Frase := To_Unbounded_String("Referencia X: " & Float'Image(rx) & " [m]" & ASCII.LF &
                                   "Referencia Y: " & Float'Image(ry) & " [m]" & ASCII.LF &
                                   "Velocidad: " & Float'Image(v_real) & " [m/s]" & ASCII.LF &
                                   "Posicion X: " & Float'Image(pos_x) & " [m]" & ASCII.LF &
                                   "Posicion Y: " & Float'Image(pos_y) & " [m]" & ASCII.LF &
                                   "Error X: " & Float'Image(e_x) & " [m]" & ASCII.LF &
                                   "Error Y: " & Float'Image(e_y) & " [m]" );

      --  Frase2 := To_Unbounded_String(Float'Image(Float(To_Duration(t-t0))) & "," &Float'Image(psi_real * 180.0 / Ada.Numerics.Pi) &
      --   "," & Float'Image(v_real) & "," & Float'Image(Sqrt(e_x**2 + e_y**2)) &
      --    "," & Float'Image(pos_x) & "," & Float'Image(pos_y));

      ---- Mostrar por pantalla---
      Pantalla.Entrar(Frase);
      delay 0.005;
      Pantalla.Salir;

      --  ---- Guardar en archivo ----
      --  Almacenamiento.Escribir(Frase2);
      --  delay 0.005;
      --  Almacenamiento.Salir;

      --- Guardar los datos en el archivo ----
      Guardar.GuardarM1(t, t0, v_real, e_x, e_y, pos_x, pos_y);
      delay 0.005;


      t := t + Ts;
      exit when t >= tfin;
      delay until t;
      end loop;
   end Control_M1;

    ---Implementación de la tarea Control_M2--
    task body Control_M2 is

      tsim       : constant Positive := 200;
      T_muestreo : constant Float    := 0.5;
      t, t0, tfin  : Time;
      Ts : Time_Span := Milliseconds (Integer (T_muestreo * 1_000.0)); -- 0.5 s

      ----------------- Parametros de control -----------------
      rx, ry, pos_x, pos_y, ori_theta, e_x, e_y, v_ref, v_real, beta, e_theta, psi_real, 
      K_nu, K_po, Long: Float;
      Ngrupo : Integer;
      CpC25 : constant Float := -0.4 / 24.0;
      type Target_X is array (1 .. 6) of Float;  -- Referencia en X
      type Target_Y is array (1 .. 6) of Float;  -- Referencia en Y

      ----------------- Definición de coordenadas ----------------- 
      Target_X_r : constant Target_X :=  (100.0, 300.0, 400.0, 300.0, 100.0, 0.0);
      Target_Y_r : constant Target_Y :=  (100.0, 150.0, 100.0, 150.0, 250.0, 0.0);

      ------------------ Variables de simulación ----------------- 
      Control_Robot : AgriCobIoT;
      M : Robot_State;

      ------------------Variable de escritura texto---------------
      Log_File : File_Type; -- Define Log_File como una variable de escritura
      c : Integer; --Contador error angulo  
      Frase : Unbounded_String; -- Variable para escritura 

     begin
               ------------- Inicialización de parametros ----------------- 
      -- IMPORTANTE: DEFINIR NUMERO DE GRUPO
      Ngrupo := 19; -- Grupo Nº 19
      Long := CpC25 * Float(Ngrupo - 1) + 0.8;
      M.L := Long; -- Longitud del robot
      M.Ts := T_muestreo; -- Tiempo de muestreo
      Initialize_Robot (Control_Robot, M); 

      ----------------- Variables temporales ----------------- 
      t    := Clock;
      t0   := Clock;
      tfin := t0 + Seconds (tsim);

      ----------------- Iniciación de variables ----------------- 
      rx         := 0.0;
      ry         := 0.0;
      pos_x      := 0.0;
      pos_y      := 0.0;
      ori_theta  := 0.0;
      e_x        := 0.0;
      e_y        := 0.0;
      v_ref      := 0.0;
      v_real     := 0.0;
      beta       := 0.0;
      e_theta    := 0.0;
      psi_real   := 0.0;
      K_nu       := 1.0;  -- Ganancia para el control de velocidad
      K_po       := 5.0;  -- Ganancia para el control de ángulo
      c          := 0;    --Contador= 0


      loop

      Tarjeta.Usar;
     ----Calculo referencia------
          if t >= t0 + Seconds (5) then
         rx := Target_X_r(1);
         ry := Target_Y_r(1);
         if t >= t0 + Seconds (50) then
            rx := Target_X_r(2);
            ry := Target_Y_r(2);
         end if;
         if t >= t0 + Seconds (70) then
            rx := Target_X_r(3);
            ry := Target_Y_r(3);
         end if;
         if t >= t0 + Seconds (100) then
            rx := Target_X_r(4);
            ry := Target_Y_r(4);
         end if;
         if t >= t0 + Seconds (125) then
            rx := Target_X_r(5);
            ry := Target_Y_r(5);
         end if;
         if t >= t0 + Seconds (150) then
            rx := Target_X_r(6);
            ry := Target_Y_r(6);
         end if;
      end if;
       ---------------  LECTURA DE DATOS  -------------------------
      pos_x       := x (Control_Robot);
      pos_y       := y (Control_Robot);
      ori_theta   := theta (Control_Robot);
      delay 0.250;
       -----------------  REFERENCIA  -------------------------
      e_x     := rx - pos_x;
      e_y     := ry - pos_y;

       -- Cálculo del ángulo deseado (beta) usando atan2
         if e_x = 0.0 and e_y = 0.0 then
             beta := 0.0; 
         else
             beta := Arctan(Y => e_y, X => e_x);
         end if;

         -- Calcular el error angular
         e_theta := beta - ori_theta;

          -- Normalización del error angular al rango [-π, π]
         e_theta := e_theta - 2.0 * Ada.Numerics.Pi * Float'Floor((e_theta + Ada.Numerics.Pi) / (2.0 * Ada.Numerics.Pi));

        --------------  CONTROL DIRECCIÓN  -------------------------
         psi_real := K_po * e_theta;

          -- Límite físico de giro
         psi_real := Float'Min(psi_real, Ada.Numerics.Pi / 4.0);  -- Límite superior (45 grados)
         psi_real := Float'Max(psi_real, -Ada.Numerics.Pi / 4.0); -- Límite inferior (-45 grados)
         delay 0.020;
          --------------  ESCRITURA DE DATOS  -------------------------
         psi(Control_Robot, psi_real);
         delay 0.010;

         Tarjeta.Salir;
         Frase := To_Unbounded_String("Referencia X: " & Float'Image(rx) & " [m]" & ASCII.LF &
                                       "Referencia Y: " & Float'Image(ry) & " [m]" & ASCII.LF &
                                       "Angulo deseado: " & Float'Image(beta) & " [rad]" & ASCII.LF &
                                       "Angulo de giro: " & Float'Image(psi_real) & " [rad]" & ASCII.LF &
                                      "Posicion X: " & Float'Image(pos_x) & " [m]" & ASCII.LF &
                                      "Posicion Y: " & Float'Image(pos_y) & " [m]" & ASCII.LF &
                                      "Error X: " & Float'Image(e_x) & " [m]" & ASCII.LF &
                                       "Error Y: " & Float'Image(e_y) & " [m]" & ASCII.LF &
                                       "Error de direccion: " & Float'Image(e_theta) & " [rad]");

      ---- Mostrar por pantalla---
      Pantalla.Entrar(Frase);
      delay 0.005;
      Pantalla.Salir;

      ---- Guardar en archivo ----
      Guardar.GuardarM2(psi_real);
      delay 0.005;
      Guardar.Escribir;
      delay 0.005;
                        



          t := t + Ts;
         exit when t >= tfin;
         delay until t;
      end loop;

     end Control_M2;





begin
   null;
end tareas;