import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1 , 'B' : 2 , 'X': 3 , 'Z': 4}

class SerialControllerInterface:

    # //                PROTOCOLO
	# // ----------------------------------------------
	# // tipo de dado -> A (analogico) D (Digital)
	# // id_botão     -> botão azul 'A'   botão verde 'B'
	# // status botão -> '1' pressionado  '0' soltou
	
	# //                HANDSHAKE
	# // --------------------------------------------- 
	# // tipo = 'H'
	# // id_botao = '0'
	# // status = '0' (pedido para desligar conexao) , status = '1' pedido para ligar conexao

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
        logging.info("----------------------")
        
        data_type = self.ser.read()
        data_dummy = self.ser.read()
        data_status = self.ser.read()

        if (data_type == b'H'):    
            logging.info("-- HANDSHAKE CONNECTED--")
            self.connected  =True

    def input_analog_action(self,speed_state):
        if(speed_state == 'a'):
            logging.info("\nAcelera\n")
            self.j.set_button(self.mapping.button['X'], 1)  
        elif(speed_state == 'd'):
            logging.info("\nRe\n")
            self.j.set_button(self.mapping.button['Z'], 1)
        else:
            logging.info("Para")
            self.j.set_button(self.mapping.button['X'], 0)
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

    def update(self):

        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()

        data_type = self.ser.read()
        id_state = self.ser.read()
        status = self.ser.read()
        
        # Pedido de desconexao:
        if(data_type == b'H' and status == b'0'):
                logging.info("--- DESCONNECTED ---")
                self.connected = False

        # Digital value
        if data_type == b'D':
            
            if(id_state == b'A'):
                self.input_digital_action('A', status)
            if(id_state == b'B'):
                self.input_digital_action('B', status)
        print(data_type)

        # Analogic value
        if data_type == b'A':
            print(id_state)
            if(id_state == b'a'):
                self.input_analog_action('a')
            elif(id_state == b'd'):
                self.input_analog_action('d')
            elif(id_state == b's'):
                self.input_analog_action('s')
                
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
            print("ESPERA update")
            controller.update()
        else:
            print("ESPERA HANDSHAKE")
            controller.handshake()