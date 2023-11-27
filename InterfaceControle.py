import serial
import re
from serial.tools import list_ports
from tkinter import *
from tkinter import messagebox

class InterfaceControle:
    def __init__(self, master):
        self.master = master
        master.title("Controle do Robô Manipulador")

        self.arduino = None
        self.porta_serial_padrao = StringVar(value='COM3')

        # Lista de portas disponíveis
        self.portas_disponiveis = self.listar_portas()

        # Drop-down para seleção da porta
        self.menu_portas = OptionMenu(master, self.porta_serial_padrao, *self.portas_disponiveis)
        self.menu_portas.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        # Botão de conexão
        self.botao_conectar = Button(master, text="Conectar", command=self.conectar)
        self.botao_conectar.grid(row=0, column=1, padx=10, pady=10, sticky="e")

        # Botão Homing
        self.botao_homing = Button(master, text="Home", command=self.enviar_homing)
        self.botao_homing.grid(row=1, column=0, columnspan=2, pady=10)

        # Elemento de texto para comandos
        self.entry_comandos = Entry(master)
        self.entry_comandos.grid(row=2, column=0, padx=10, pady=10, sticky="w")

        # Botão para enviar comandos
        self.botao_enviar_comando = Button(master, text="Enviar Comando", command=self.enviar_comando_digitado)
        self.botao_enviar_comando.grid(row=2, column=1, padx=10, pady=10, sticky="e")

        # Criação de barras de rolagem para cada junta
        self.elementos_interface = [
            self.criar_barra_rolagem(1, -140, 120),
            self.criar_barra_rolagem(2, 0, 180),
            self.criar_barra_rolagem(3, -90, 90)
        ]

        # Label para exibir o status da conexão
        self.status_label = Label(master, text="Desconectado", font=("Arial", 12, "bold"))
        self.status_label.grid(row=3, column=0, columnspan=2, pady=10)

    def listar_portas(self):
        return [port.device for port in list_ports.comports()]

    def conectar(self):
        try:
            # Tentar conectar na porta selecionada
            self.arduino = serial.Serial(self.porta_serial_padrao.get(), 9600)
            # Atualizar o estado dos elementos da interface
            for elemento in self.elementos_interface:
                elemento.config(state=NORMAL)
            self.botao_conectar.config(state=DISABLED)
            self.botao_homing.config(state=NORMAL)
            self.botao_enviar_comando.config(state=NORMAL)
            self.status_label.config(text=f"Conectado à porta {self.porta_serial_padrao.get()}", fg="green")
        except serial.SerialException:
            self.status_label.config(text="Erro ao conectar. Verifique a porta selecionada.", fg="red")

    def enviar_comando(self, junta, valor):
        comando = f'{junta}:{valor}'
        self.arduino.write(comando.encode())

    def enviar_homing(self):
        # Enviar os ângulos de homing: 0, 0, 0
        self.enviar_comando(1, 0)
        self.enviar_comando(2, 0)
        self.enviar_comando(3, 0)

    def validar_comando(self, comando):
        formato_1 = re.compile(r'^D T1:\d+ T2:\d+ T3:\d+$')
        formato_2 = re.compile(r'^I X\d+ Y\d+ Z\d+$')

        if formato_1.match(comando) or formato_2.match(comando):
            return True
        else:
            return False

    def enviar_comando_digitado(self):
        comando_digitado = self.entry_comandos.get()
        if self.validar_comando(comando_digitado):
            self.arduino.write(comando_digitado.encode())
            print(f"Comando válido: {comando_digitado}")
        else:
            messagebox.showerror("Erro", "Formato de comando inválido. Use 'D T1:xx T2:yy T3:zz' ou 'I Xnn Ynn Znn'")

    def criar_barra_rolagem(self, junta, limite_min, limite_max):
        scrollbar = Scale(self.master, from_=limite_min, to=limite_max, orient=HORIZONTAL, label=f'Junta {junta}', length=300, state=DISABLED)
        scrollbar.grid(row=junta+3, column=0, columnspan=2, pady=5)
        scrollbar.bind("<B1-Motion>", lambda event, j=junta: self.atualizar_junta(j, scrollbar.get()))
        return scrollbar

    def atualizar_junta(self, junta, valor):
        self.enviar_comando(junta, valor)

if __name__ == "__main__":
    root = Tk()
    interface = InterfaceControle(root)
    root.mainloop()

    if interface.arduino is not None and interface.arduino.is_open:
        interface.arduino.close()
