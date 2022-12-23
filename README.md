##  🎮️  Mario Kart Controller

Um controle que utiliza o microcontrolador SAME70 que, em conjunto com um código em python, realiza a comunicação com o jogo emulado no computador, Mario Kart double dash, por meio de bluetooth.

Para o melhor uso recomenda-se utilizar: 

- **Programação no microcontrolador** : [Microship Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)

- **Emulador** : [Dolphin](https://br.dolphin-emu.org/)

<div align="center">
<img alt="jogo mario" src = "img_README/BelovedSeriousKouprey-size_restricted.gif" width="500"></img>
</div>

<br></br>

### 📌️ Descição de Comandos e Feedbacks 

- Acelearar para frente ou para trás: <b>Potenciômetro</b> 

- Virar para os lados : <b> IMU </b>

- Drift : <b>Push button verde</b>

- Usar itens : <b>Push button azul</b>

- Selecionar e percorrer menus e configuracoes do jogo : <b>Joypad lateral</b>

- Botao liga/desliga : <b>Mini illuminated pushbutton</b>
<br></br>

- Botao verde pressionado : <b>LED verde</b>

- Botao azul pressionado : <b>LED azul</b>

- Controle ligado/conectado: <b>LED interno do botão power</b>
<br></br>

### ⚙️ Funcionamento

Para começar o uso do projeto é necessário carregar o código contido no arquivo `firmware/src/main.c` no microcontrolador SAME70.
Com o código carregado no microcontrolador, precisamos ativar o código python responsável pela leitura dos sinais do controle.

Para rodar o código python, siga as seguintes instrições:

1 - Abra um terminal dentro da pasta `python`

2 - Identifique em que porta COM do computador o microcontrolador esta conectado. Em caso de dúvida, [clique aqui](https://answers.microsoft.com/pt-br/windows/forum/all/cad%C3%AA-as-portas-com-e-lpt-do-windows-10/aeea1cf3-ac8b-4fa0-9614-80175eeeeb28).

3 - Rode o código python no terminal da seguinte forma:

```bash
./game_controller.py <PORTA COM> -b 115200 -d

```

Teste os analógicos e botões do controle e veja se alguma informação aparece no terminal. Caso sim, o controle está pronto para jogar.
Inicie o jogo no emulador e divirta-se!

Para ver o controle em ação acesse a playlist : [clique aqui](https://alinsperedu-my.sharepoint.com/personal/lorrancml_al_insper_edu_br/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Florrancml%5Fal%5Finsper%5Fedu%5Fbr%2FDocuments%2Fmario%20kart&ga=1)
