(* ============================================================
  KitchenWatch — пример управления по WebSocket на FreePascal.

  Программа подключается к WebSocket-серверу устройства (порт 81),
  читает приветствие status=connected и снимок состояния, отправляет
  команды; для get/status печатает ответ устройства.

  Сборка (FPC 3.2.2+):
      fpc -Mobjfpc tools/send_command.pas

  Запуск:
      tools/send_command.exe [IP] [порт] [команда [параметры...]]

  Примеры:
      tools/send_command.exe
      tools/send_command.exe 192.168.1.55
      tools/send_command.exe 192.168.1.55 81 get
      tools/send_command.exe 192.168.1.55 81 next
      tools/send_command.exe 192.168.1.55 81 frequency 103.9
      tools/send_command.exe 192.168.1.55 81 clock_color 0 255 0

  Команды: get, status, next, prev, toggle, frequency, clock_color,
  freq_color, volume, brightness, color, on, off, auto.
  Для get и status (в т.ч. в виде готового JSON) печатается ответ —
  снимок состояния {"cmd":"state",...}.
  Вместо имени команды можно передать готовый JSON (аргумент,
  начинающийся с символа «{»), например из cmd.exe:
      send_command.exe 192.168.1.55 81 "{\"cmd\":\"next\"}"

  Без команды отправляется демонстрационная последовательность.
  Если параметры не указаны, используется 192.168.1.55:81.

  Зависимости — только стандартные модули FPC:
      ssockets (fcl-net), base64 (fcl-base), sha1 (hash).
  Поддерживается только ws:// (без TLS) — прошивка работает именно так.

  Примечание: для краткости пример рассчитан на короткие текстовые
  кадры (как в протоколе KitchenWatch); фрагментированные сообщения
  и большие кадры не обрабатываются.
  ============================================================ *)

program send_command;

{$mode objfpc}{$H+}
{$codepage utf8}

uses
  SysUtils, Classes, ssockets, base64, sha1;

const
  DEFAULT_HOST = '192.168.1.55';
  DEFAULT_PORT = 81;

  // Магическая строка из RFC 6455 для проверки Sec-WebSocket-Accept.
  WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

  // Коды опкодов WebSocket.
  OP_TEXT  = $1;
  OP_CLOSE = $8;
  OP_PING  = $9;
  OP_PONG  = $A;

type
  TWSClient = class
  private
    FSocket: TInetSocket;
    procedure SendFrame(AOpcode: Byte; const APayload: RawByteString);
    function ReadFrame(out AOpcode: Byte; out APayload: RawByteString): Boolean;
  public
    constructor Create(const AHost: string; APort: Word);
    destructor Destroy; override;
    procedure SendText(const AText: RawByteString);
    function ReceiveText(out AText: RawByteString): Boolean;
  end;

{ Возвращает SHA-1 строки в виде «сырых» 20 байт. }
function Sha1Raw(const S: RawByteString): RawByteString;
var
  Digest: TSHA1Digest;
begin
  Digest := SHA1String(S);
  SetLength(Result, SizeOf(Digest));
  Move(Digest[0], Result[1], SizeOf(Digest));
end;

{ Значение HTTP-заголовка из ответа на рукопожатие (без учёта регистра). }
function HeaderValue(const AResponse, AName: string): string;
var
  Lines: TStringList;
  I: Integer;
  Prefix: string;
begin
  Result := '';
  Prefix := LowerCase(AName) + ':';
  Lines := TStringList.Create;
  try
    Lines.Text := AResponse;
    for I := 0 to Lines.Count - 1 do
      if Pos(Prefix, LowerCase(Lines[I])) = 1 then
        Exit(Trim(Copy(Lines[I], Length(Prefix) + 1, MaxInt)));
  finally
    Lines.Free;
  end;
end;

constructor TWSClient.Create(const AHost: string; APort: Word);
var
  KeyBytes: array[0..15] of Byte;
  Key, KeyB64, Request, Response, Expected, StatusLine: RawByteString;
  Buf: Byte;
  I, HeaderEnd: Integer;
begin
  inherited Create;

  // 16 случайных байт -> Sec-WebSocket-Key (base64).
  for I := 0 to High(KeyBytes) do
    KeyBytes[I] := Random(256);
  SetLength(Key, SizeOf(KeyBytes));
  Move(KeyBytes[0], Key[1], SizeOf(KeyBytes));
  KeyB64 := EncodeStringBase64(Key);

  // Внимание: при AHandler = Nil конструктор TInetSocket сам выполняет
  // подключение, поэтому отдельный вызов Connect не нужен.
  FSocket := TInetSocket.Create(AHost, APort);
  FSocket.IOTimeout := 5000; // таймаут чтения, мс

  try
    Request := 'GET / HTTP/1.1' + #13#10 +
               'Host: ' + AHost + ':' + IntToStr(APort) + #13#10 +
               'Upgrade: websocket' + #13#10 +
               'Connection: Upgrade' + #13#10 +
               'Sec-WebSocket-Key: ' + KeyB64 + #13#10 +
               'Sec-WebSocket-Version: 13' + #13#10 +
               #13#10;
    FSocket.WriteBuffer(Request[1], Length(Request));

    // Читаем заголовки ответа до пустой строки.
    Response := '';
    repeat
      FSocket.ReadBuffer(Buf, 1);
      Response := Response + Chr(Buf);
    until (Length(Response) >= 4) and
          (Copy(Response, Length(Response) - 3, 4) = #13#10#13#10);

    HeaderEnd := Pos(#13#10, Response);
    if HeaderEnd > 0 then
      StatusLine := Copy(Response, 1, HeaderEnd - 1)
    else
      StatusLine := Response;
    if Pos('101', StatusLine) = 0 then
      raise Exception.Create('WebSocket handshake failed: ' + StatusLine);

    // Проверяем, что сервер правильно посчитал Sec-WebSocket-Accept.
    // По RFC 6455 хешируется base64-строка ключа (значение заголовка),
    // а не исходные 16 случайных байт.
    Expected := EncodeStringBase64(Sha1Raw(KeyB64 + WS_GUID));
    if HeaderValue(Response, 'Sec-WebSocket-Accept') <> Expected then
      raise Exception.Create('WebSocket handshake failed: bad Sec-WebSocket-Accept');
  except
    FreeAndNil(FSocket);
    raise;
  end;
end;

destructor TWSClient.Destroy;
begin
  if Assigned(FSocket) then
  begin
    // Кадр Close (пустой payload), затем закрываем сокет.
    try
      SendFrame(OP_CLOSE, '');
    except
      // Соединение могло уже закрыться — это не ошибка.
    end;
    FreeAndNil(FSocket);
  end;
  inherited Destroy;
end;

{ Отправляет один кадр. Клиентские кадры всегда маскируются (RFC 6455). }
procedure TWSClient.SendFrame(AOpcode: Byte; const APayload: RawByteString);
var
  Header: array[0..13] of Byte;
  Mask: array[0..3] of Byte;
  HeaderLen, I: Integer;
  Len: SizeInt;
  Data: RawByteString;
begin
  Len := Length(APayload);
  Header[0] := $80 or AOpcode; // FIN = 1

  if Len < 126 then
  begin
    Header[1] := $80 or Byte(Len);
    HeaderLen := 2;
  end
  else if Len <= $FFFF then
  begin
    Header[1] := $80 or 126;
    Header[2] := (Len shr 8) and $FF;
    Header[3] := Len and $FF;
    HeaderLen := 4;
  end
  else
  begin
    Header[1] := $80 or 127;
    for I := 0 to 7 do
      Header[2 + I] := (QWord(Len) shr ((7 - I) * 8)) and $FF;
    HeaderLen := 10;
  end;

  for I := 0 to 3 do
    Mask[I] := Random(256);
  Move(Mask[0], Header[HeaderLen], SizeOf(Mask));
  Inc(HeaderLen, SizeOf(Mask));

  Data := APayload;
  for I := 1 to Len do
    Data[I] := Chr(Ord(Data[I]) xor Mask[(I - 1) mod 4]);

  FSocket.WriteBuffer(Header[0], HeaderLen);
  if Len > 0 then
    FSocket.WriteBuffer(Data[1], Len);
end;

{ Читает один кадр. Возвращает False для кадра Close. }
function TWSClient.ReadFrame(out AOpcode: Byte; out APayload: RawByteString): Boolean;
var
  B0, B1: Byte;
  Ext: array[0..7] of Byte;
  Mask: array[0..3] of Byte;
  Len: QWord;
  I: Integer;
  Masked: Boolean;
begin
  FSocket.ReadBuffer(B0, 1);
  FSocket.ReadBuffer(B1, 1);

  AOpcode := B0 and $0F;
  Masked  := (B1 and $80) <> 0;
  Len     := B1 and $7F;

  if Len = 126 then
  begin
    FSocket.ReadBuffer(Ext[0], 2);
    Len := (QWord(Ext[0]) shl 8) or Ext[1];
  end
  else if Len = 127 then
  begin
    FSocket.ReadBuffer(Ext[0], 8);
    Len := 0;
    for I := 0 to 7 do
      Len := (Len shl 8) or Ext[I];
  end;

  if Len > 1024 * 1024 then
    raise Exception.CreateFmt('WebSocket frame too large: %d bytes', [Len]);

  if Masked then
    FSocket.ReadBuffer(Mask[0], SizeOf(Mask));

  SetLength(APayload, SizeInt(Len));
  if Len > 0 then
    FSocket.ReadBuffer(APayload[1], SizeInt(Len));

  if Masked then
    for I := 1 to SizeInt(Len) do
      APayload[I] := Chr(Ord(APayload[I]) xor Mask[(I - 1) mod 4]);

  Result := AOpcode <> OP_CLOSE;
end;

procedure TWSClient.SendText(const AText: RawByteString);
begin
  SendFrame(OP_TEXT, AText);
end;

{ Читает текстовое сообщение. Кадры Ping подтверждаются Pong. }
function TWSClient.ReceiveText(out AText: RawByteString): Boolean;
var
  Opcode: Byte;
  Payload: RawByteString;
begin
  Result := False;
  AText  := '';
  while True do
  begin
    if not ReadFrame(Opcode, Payload) then
      Exit(False); // Close
    case Opcode of
      OP_TEXT:
        begin
          AText := Payload;
          Exit(True);
        end;
      OP_PING:
        SendFrame(OP_PONG, Payload);
      // Остальные opcode (pong, binary) в этом примере игнорируются.
    end;
  end;
end;

{ Отправляет одну команду, печатая её в консоль. }
procedure SendCommand(Client: TWSClient; const AJson: string);
begin
  WriteLn('>> ', AJson);
  Client.SendText(AJson);
  Sleep(100); // даём устройству обработать команду (поиск станции блокирующий)
end;

{ Отправляет команду и печатает ответ сервера (для get / status). }
procedure SendCommandWithReply(Client: TWSClient; const AJson: string);
var
  Reply: RawByteString;
begin
  WriteLn('>> ', AJson);
  Client.SendText(AJson);
  if Client.ReceiveText(Reply) then
    WriteLn('Server: ', Reply);
end;

{ Нужен ли ответ на команду: get / status именем или готовым JSON. }
function WantsReply(const ACmd, AJson: string): Boolean;
begin
  Result := SameText(ACmd, 'get') or SameText(ACmd, 'status') or
            (Pos('"cmd":"get"', AJson) > 0) or
            (Pos('"cmd":"status"', AJson) > 0);
end;

{ Обязательный параметр командной строки (нумерация как в ParamStr). }
function NeedParam(AIndex: Integer; const AUsage: string): string;
begin
  if ParamCount < AIndex then
    raise Exception.Create('Missing parameter, usage: ' + AUsage);
  Result := ParamStr(AIndex);
end;

{ Строит JSON по короткому синтаксису командной строки:
  «next», «frequency 103.9», «clock_color 0 255 0» и т.п. }
function BuildCommand(const ACmd: string): string;
var
  ColorUsage: string;
begin
  if SameText(ACmd, 'frequency') then
    Result := '{"cmd":"frequency","value":' + NeedParam(4, 'frequency <MHz>') + '}'
  else if SameText(ACmd, 'volume') or SameText(ACmd, 'brightness') then
    Result := '{"cmd":"' + ACmd + '","value":' + NeedParam(4, ACmd + ' <value>') + '}'
  else if SameText(ACmd, 'clock_color') or SameText(ACmd, 'freq_color') or
          SameText(ACmd, 'color') then
  begin
    ColorUsage := ACmd + ' <r> <g> <b>';
    Result := '{"cmd":"' + ACmd +
              '","r":' + NeedParam(4, ColorUsage) +
              ',"g":' + NeedParam(5, ColorUsage) +
              ',"b":' + NeedParam(6, ColorUsage) + '}';
  end
  else
    Result := '{"cmd":"' + ACmd + '"}';
end;

var
  Host: string;
  Port: Word;
  Json, Reply: RawByteString;
  Client: TWSClient;
  I: Integer;
begin
  Randomize;

  Host := DEFAULT_HOST;
  Port := DEFAULT_PORT;
  Json := '';

  if ParamCount >= 1 then Host := ParamStr(1);
  if ParamCount >= 2 then Port := Word(StrToIntDef(ParamStr(2), DEFAULT_PORT));
  if ParamCount >= 3 then
  begin
    // Аргумент с «{» считается готовым JSON и отправляется как есть.
    if (Length(ParamStr(3)) > 0) and (ParamStr(3)[1] = '{') then
      Json := ParamStr(3)
    else
      Json := BuildCommand(ParamStr(3));
  end;

  Client := TWSClient.Create(Host, Port);
  try
    WriteLn('Connected to ws://', Host, ':', Port);

    // При подключении сервер отправляет два сообщения:
    // {"status":"connected"} и {"cmd":"state",...} — снимок настроек.
    for I := 1 to 2 do
      if Client.ReceiveText(Reply) then
        WriteLn('Server: ', Reply);

    if Json <> '' then
    begin
      if WantsReply(ParamStr(3), Json) then
        SendCommandWithReply(Client, Json)
      else
        SendCommand(Client, Json);
    end
    else
    begin
      // Демонстрационная последовательность: сначала запрос настроек,
      // затем частота, станции, переключение индикации и цвета цифр.
      SendCommandWithReply(Client, '{"cmd":"get"}');
      SendCommand(Client, '{"cmd":"frequency","value":103.9}');
      SendCommand(Client, '{"cmd":"next"}');
      SendCommand(Client, '{"cmd":"prev"}');
      SendCommand(Client, '{"cmd":"toggle"}');
      SendCommand(Client, '{"cmd":"clock_color","r":0,"g":255,"b":0}');
      SendCommand(Client, '{"cmd":"freq_color","r":0,"g":0,"b":255}');
    end;
  finally
    Client.Free;
  end;

  WriteLn('Done.');
end.
