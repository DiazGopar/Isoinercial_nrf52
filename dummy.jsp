public scan():Observable<any> {
    
    return this.startScan({
      services: ['ad390538-d18b-11e6-bf26-cec0c932ce01'],
      "allowDuplicates": false,
      "matchNum": BluetoothMatchNum.MATCH_NUM_ONE_ADVERTISEMENT
    });

  }

  public conectar(timeout:number, address:string) {
    let timer = null;
    if(timeout > 0) {
      timer = setTimeout(() => {
        console.log("Timeout");
        this.deviceConnect.unsubscribe();
        this.close({address: address});
      }, timeout);
    }
    
    this.deviceConnect = this.connect({address: address}).subscribe(device => {
      console.log("Device",device);   
      if(timeout > 0) clearTimeout(timer);
      this.deviceId = device;

      this.discover({ address: this.deviceId.address, clearCache: false })
      .then((devDiscover) => {
        console.log("Services", devDiscover);
      }).catch((err) => console.log(err));

      this._deviceChange.next(true);
      
    }, (err) => {
      console.log("Error conexion", err);
      this.deviceConnect.unsubscribe();
      this.close({address: address});
    });

  }


this.subscribe({
      address: this.deviceId.address,
      service: "AD390538-D18B-11E6-BF26-CEC0C932CE01",
      characteristic: "AD390539-D18B-11E6-BF26-CEC0C932CE01"
    }).subscribe((valorRead:any) => {
      
        if(valorRead.status === "subscribedResult") {
        let valueRead = this.encodedStringToBytes(valorRead.value);
       
        let tiempo = new DataView(valueRead.buffer, 0, 4).getInt32(0, true);
        let pos = new DataView(valueRead.buffer, 4, 2).getInt16(0, true);

