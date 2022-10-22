import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1 , 'B' : 2 , 'ENTERandX': 3 , 'Z': 4 ,'LEFT': 6 , 'RIGHT': 7 , 'UP': 8 , 'DOWN': 9}

class SerialControllerInterface:

    # //                PROTOCOLO
	# // ----------------------------------------------
	# // tipo de dado -> A (analogico) D (Digital) I (IMU)
	# // id_botão     -> botão azul 'A'   botão verde 'B'
	# // status botão -> '1' pressionado  '0' soltou
	
	# //                HANDSHAKE
	# // --------------------------------------------- 
	# // tipo = 'H'
	# // id_botao = '0'
	# // status = '0' (pedido para desligar conexao) , status = '1' pedido para ligar conexao

    # //                VJOY DATA
	# // ---------------------------------------------
	# // tipo = 'J'
	# // id_botao = info eixo x
	# // status = info eixo y

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'
        self.connected = False

    def handshake(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.info(self.incoming)
     
        h0 = self.ser.read()
        h1 = self.ser.read()
        h2 = self.ser.read()

        if (h0 == b'H'):    
            logging.info("\n-- HANDSHAKE CONNECTED--\n")
            self.ser.write('R'.encode(encoding = 'ascii'))
            logging.info("\n-- SEND ACK --\n")
            self.connected  =True

    def input_analog_action(self,speed_state):
        if(speed_state == 'a'):
            logging.info("\nAcelera\n")
            self.j.set_button(self.mapping.button['ENTERandX'], 1)  
        elif(speed_state == 'd'):
            logging.info("\nRe\n")
            self.j.set_button(self.mapping.button['Z'], 1)
        else:
            logging.info("Para")
            self.j.set_button(self.mapping.button['ENTERandX'], 0)
            self.j.set_button(self.mapping.button['Z'], 0)

    def input_digital_action(self,buttonId, status):
        logging.info("Pressing \t")
        logging.info(buttonId)

        if(status == b'1'):
            logging.info("- KeyDown \n")
            self.j.set_button(self.mapping.button[buttonId], 1)  
        else:    
            logging.info("- KeyUp \n")
            self.j.set_button(self.mapping.button[buttonId], 0)  

    def input_vjoy_values(self, x , y):
        logging.info("\n ---- JOYSTICK ---- \n")
        logging.info(x)
        logging.info(y)
        print('\n')
        if x == b'0':
            self.j.set_button(self.mapping.button['LEFT'], 0)  
            self.j.set_button(self.mapping.button['RIGHT'], 0) 
        if y == b'0':
            self.j.set_button(self.mapping.button['UP'], 0)              
            self.j.set_button(self.mapping.button['DOWN'], 0)  
            
        if x == b'e':
            self.j.set_button(self.mapping.button['LEFT'], 1)   
        if x == b'd':
            self.j.set_button(self.mapping.button['RIGHT'], 1)   
        if y== b'c':
            self.j.set_button(self.mapping.button['UP'], 1) 
        if y== b'b':
            self.j.set_button(self.mapping.button['DOWN'], 1) 

    def input_imu_action(self,speed_state):
        print("\n ---- IMU ----\n")
        if(speed_state == 'r'):
            logging.info("\nRight\n")
            self.j.set_button(self.mapping.button['RIGHT'], 1)  
        elif(speed_state == 'l'):
            logging.info("\nLeft\n")
            self.j.set_button(self.mapping.button['LEFT'], 1)
        else:
            logging.info("Para")
            self.j.set_button(self.mapping.button['RIGHT'], 0)
            self.j.set_button(self.mapping.button['LEFT'], 0)

    def update(self):

        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()

        h0 = self.ser.read()
        h1 = self.ser.read()
        h2 = self.ser.read()
        
        # Pedido de desconexao:
        if(h0 == b'H' and h2 == b'0'):
                logging.info("--- DESCONNECTED ---")
                self.connected = False

        # Digital value
        if h0 == b'D':
            if(h1 == b'A'):
                self.input_digital_action('A', h2)
            if(h1 == b'B'):
                self.input_digital_action('B', h2)
            if(h1 == b'Y'):
                self.input_digital_action('ENTERandX', h2)

        # Analogic value
        if h0 == b'A':
            if(h1 == b'a'):
                self.input_analog_action('a')
            elif(h1 == b'd'):
                self.input_analog_action('d')
            elif(h1 == b's'):
                self.input_analog_action('s')

        # Joystick
        if h0 == b'J':
            #                 eixo_x    eixo_y
             self.input_vjoy_values(h1, h2)

        # IMU value
        if h0 == b'I':
            if(h1 == b'r'):
                self.input_imu_action('r')
            elif(h1 == b'l'):
                self.input_imu_action('l')
            elif(h1 == b's'):
                self.input_imu_action('s')
                
        self.incoming = self.ser.read()

class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        
        if(controller.connected):
            controller.update()
        else:
            controller.handshake()