//
//  SerialViewController.swift
//  HM10 Serial
//
//  Created by Alex on 10-08-15.
//  Copyright (c) 2015 Balancing Rock. All rights reserved.
//

import UIKit
import CoreBluetooth
import QuartzCore

/// The option to add a \n or \r or \r\n to the end of the send message
enum MessageOption: Int {
    case noLineEnding,
         newline,
         carriageReturn,
         carriageReturnAndNewline
}

/// The option to add a \n to the end of the received message (to make it more readable)
enum ReceivedMessageOption: Int {
    case none,
         newline
}

var timer = Timer()
var location = CGPoint(x: 0, y: 0)
var joyXvalue = CGFloat(0)
var joyYvalue = CGFloat(0)
var joyRvalue = CGFloat(0)
var joyPhivalue = CGFloat(0)
let baseVerticalLine = CAShapeLayer()
let baseHorizontalLine = CAShapeLayer()
let targetVerticalLine = CAShapeLayer()
let targetHorizontalLine = CAShapeLayer()
let baseCircle = CAShapeLayer()
let targetCircle = CAShapeLayer()
let baseWidth = CGFloat(270)
let baseHeight = CGFloat(270)
let baseXOffset = CGFloat(40)
let baseYOffset = CGFloat(60)
let buttonSize = CGFloat(20)
let centerPointFeeling = CGFloat(5)

var sendLength = Int(0)
var sendAngle = Int(0)
var sendButton = Int(0)

let ACircle = CAShapeLayer()
let BCircle = CAShapeLayer()
let CCircle = CAShapeLayer()
let DCircle = CAShapeLayer()

var AState = Int(0)
var BState = Int(0)
var CState = Int(0)
var DState = Int(0)
var switchState = Int(0)
var aliveCounter = Int(0)

final class SerialViewController: UIViewController, UITextFieldDelegate, BluetoothSerialDelegate {

//MARK: IBOutlets
    
    @IBOutlet weak var mainTextView: UITextView!
    @IBOutlet weak var messageField: UITextField!
    @IBOutlet weak var bottomView: UIView!
    @IBOutlet weak var bottomConstraint: NSLayoutConstraint! // used to move the textField up when the keyboard is present
    @IBOutlet weak var barButton: UIBarButtonItem!
    @IBOutlet weak var navItem: UINavigationItem!
    @IBOutlet weak var infoText: UITextView!
    @IBAction func Switch1(_ sender: UISwitch) {
        if (sender.isOn == true){
            switchState = 1
        }
        else {
            switchState = 0
        }
    }
    

//MARK: Functions
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // init serial
        serial = BluetoothSerial(delegate: self)
        
        // UI
        mainTextView.text = ""
        reloadView()
        
        NotificationCenter.default.addObserver(self, selector: #selector(SerialViewController.reloadView), name: NSNotification.Name(rawValue: "reloadStartViewController"), object: nil)
        
        // we want to be notified when the keyboard is shown (so we can move the textField up)
        NotificationCenter.default.addObserver(self, selector: #selector(SerialViewController.keyboardWillShow(_:)), name: UIWindow.keyboardWillShowNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(SerialViewController.keyboardWillHide(_:)), name: UIWindow.keyboardWillHideNotification, object: nil)
        
        // to dismiss the keyboard if the user taps outside the textField while editing
        let tap = UITapGestureRecognizer(target: self, action: #selector(SerialViewController.dismissKeyboard))
        tap.cancelsTouchesInView = false
        view.addGestureRecognizer(tap)
        
        // style the bottom UIView
        bottomView.layer.masksToBounds = false
        bottomView.layer.shadowOffset = CGSize(width: 0, height: -1)
        bottomView.layer.shadowRadius = 0
        bottomView.layer.shadowOpacity = 0.5
        bottomView.layer.shadowColor = UIColor.gray.cgColor
        
        scheduledTimerWithTimeInterval()
        addLine(line: baseHorizontalLine, fromPoint: CGPoint(x: baseXOffset, y: baseYOffset + baseHeight/2), toPoint: CGPoint(x: baseXOffset+baseWidth, y: baseYOffset + baseHeight/2), color: UIColor.black)
        addLine(line: baseVerticalLine, fromPoint: CGPoint(x: baseXOffset+baseWidth/2, y: baseYOffset), toPoint: CGPoint(x: baseXOffset+baseWidth/2, y: baseYOffset+baseHeight), color: UIColor.black)
        addLine(line: targetHorizontalLine, fromPoint: CGPoint(x: baseXOffset, y: baseYOffset + baseHeight/2), toPoint: CGPoint(x: baseXOffset+baseWidth, y: baseYOffset + baseHeight/2), color: UIColor.red)
        addLine(line: targetVerticalLine, fromPoint: CGPoint(x: baseXOffset+baseWidth/2, y: baseYOffset), toPoint: CGPoint(x: baseXOffset+baseWidth/2, y: baseYOffset+baseHeight), color: UIColor.red)
        addCircle(circle: baseCircle, center: CGPoint(x:baseXOffset+baseWidth/2, y:baseYOffset + baseHeight/2), r: CGFloat(baseWidth/2), fill: false, color: UIColor.red)
        addCircle(circle: targetCircle, center: CGPoint(x:baseXOffset+baseWidth/2, y:baseYOffset + baseHeight/2), r: CGFloat(buttonSize), fill: true, color: UIColor.black)
        
        addCircle(circle: ACircle, center: CGPoint(x:630, y:100), r: CGFloat(30), fill: true, color: UIColor.black)
        addCircle(circle: BCircle, center: CGPoint(x:710, y:190), r: CGFloat(30), fill: true, color: UIColor.black)
        addCircle(circle: CCircle, center: CGPoint(x:630, y:280), r: CGFloat(30), fill: true, color: UIColor.black)
        addCircle(circle: DCircle, center: CGPoint(x:550, y:190), r: CGFloat(30), fill: true, color: UIColor.black)
        
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesBegan(touches, with: event)
        
        guard let touch = touches.first else {
            return
        }
        
        location = touch.location(in: self.view)
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesMoved(touches, with: event)
        
        guard let touch = touches.first else {
            return
        }
        
        location = touch.location(in: self.view)
        
        if (location.x <= 350){
        
            joyXvalue = location.x-(baseXOffset+baseWidth/2)
            joyYvalue = location.y-(baseYOffset+baseHeight/2)
            
            let joyRPhi = XY2RPhi(x: joyXvalue, y: joyYvalue)
            joyRvalue = joyRPhi.r
            joyPhivalue = joyRPhi.phi
            
            if (joyRvalue < centerPointFeeling){
                joyRvalue = 0
                joyXvalue = 0
                joyYvalue = 0
            }
            
            if (joyRvalue > baseWidth/2 - buttonSize){
                joyRvalue = baseWidth/2 - buttonSize
                let joyXY = RPhi2XY(r: joyRvalue, phi: joyPhivalue)
                joyXvalue = joyXY.x
                joyYvalue = joyXY.y
            }
            
            targetHorizontalLine.frame.origin = CGPoint(x: targetHorizontalLine.frame.origin.x, y: joyYvalue)
            targetVerticalLine.frame.origin = CGPoint(x: joyXvalue, y: targetVerticalLine.frame.origin.y)
            targetCircle.frame.origin = CGPoint(x: joyXvalue, y: joyYvalue)
            targetCircle.fillColor = UIColor.red.cgColor
            targetCircle.strokeColor = UIColor.red.cgColor

        }
        else{
            if ((location.x>=590) && (location.x<=670) && (location.y>=60) && (location.y<=140)){
                ACircle.fillColor = UIColor.red.cgColor
                ACircle.strokeColor = UIColor.red.cgColor
                AState = 1
                
                BCircle.fillColor = UIColor.black.cgColor
                BCircle.strokeColor = UIColor.black.cgColor
                CCircle.fillColor = UIColor.black.cgColor
                CCircle.strokeColor = UIColor.black.cgColor
                DCircle.fillColor = UIColor.black.cgColor
                DCircle.strokeColor = UIColor.black.cgColor
                BState = 0
                CState = 0
                DState = 0
            }
            if ((location.x>=680) && (location.x<=760) && (location.y>=140) && (location.y<=220)){
                BCircle.fillColor = UIColor.red.cgColor
                BCircle.strokeColor = UIColor.red.cgColor
                BState = 1
                
                ACircle.fillColor = UIColor.black.cgColor
                ACircle.strokeColor = UIColor.black.cgColor
                CCircle.fillColor = UIColor.black.cgColor
                CCircle.strokeColor = UIColor.black.cgColor
                DCircle.fillColor = UIColor.black.cgColor
                DCircle.strokeColor = UIColor.black.cgColor
                AState = 0
                CState = 0
                DState = 0
            }
            if ((location.x>=590) && (location.x<=670) && (location.y>=230) && (location.y<=310)){
                CCircle.fillColor = UIColor.red.cgColor
                CCircle.strokeColor = UIColor.red.cgColor
                CState = 1
                
                BCircle.fillColor = UIColor.black.cgColor
                BCircle.strokeColor = UIColor.black.cgColor
                ACircle.fillColor = UIColor.black.cgColor
                ACircle.strokeColor = UIColor.black.cgColor
                DCircle.fillColor = UIColor.black.cgColor
                DCircle.strokeColor = UIColor.black.cgColor
                BState = 0
                AState = 0
                DState = 0
            }
            if ((location.x>=520) && (location.x<=600) && (location.y>=140) && (location.y<=220)){
                DCircle.fillColor = UIColor.red.cgColor
                DCircle.strokeColor = UIColor.red.cgColor
                DState = 1
                
                BCircle.fillColor = UIColor.black.cgColor
                BCircle.strokeColor = UIColor.black.cgColor
                CCircle.fillColor = UIColor.black.cgColor
                CCircle.strokeColor = UIColor.black.cgColor
                ACircle.fillColor = UIColor.black.cgColor
                ACircle.strokeColor = UIColor.black.cgColor
                BState = 0
                CState = 0
                AState = 0
            }
            
        }
    }
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        
        joyRvalue = 0
        joyXvalue = 0
        joyYvalue = 0
        
        AState = 0
        BState = 0
        CState = 0
        DState = 0
        
        targetHorizontalLine.frame.origin = CGPoint(x: 0, y: 0)
        targetVerticalLine.frame.origin = CGPoint(x: 0, y: 0)
        targetCircle.frame.origin = CGPoint(x: 0, y: 0)
        targetCircle.fillColor = UIColor.black.cgColor
        targetCircle.strokeColor = UIColor.black.cgColor
        ACircle.fillColor = UIColor.black.cgColor
        ACircle.strokeColor = UIColor.black.cgColor
        BCircle.fillColor = UIColor.black.cgColor
        BCircle.strokeColor = UIColor.black.cgColor
        CCircle.fillColor = UIColor.black.cgColor
        CCircle.strokeColor = UIColor.black.cgColor
        DCircle.fillColor = UIColor.black.cgColor
        DCircle.strokeColor = UIColor.black.cgColor
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }
    
    func addLine(line: CAShapeLayer, fromPoint start: CGPoint, toPoint end:CGPoint, color: UIColor) {
        //let line = CAShapeLayer()
        let linePath = UIBezierPath()
        linePath.move(to: start)
        linePath.addLine(to: end)
        line.path = linePath.cgPath
        line.strokeColor = color.cgColor
        line.lineWidth = 1
        line.lineJoin = CAShapeLayerLineJoin.round
        self.view.layer.addSublayer(line)
    }
    
    func addCircle(circle: CAShapeLayer, center: CGPoint, r: CGFloat, fill: Bool, color: UIColor){
        let circlePath = UIBezierPath(arcCenter: center, radius: r, startAngle: CGFloat(0), endAngle:CGFloat(Double.pi * 2), clockwise: true)
        circle.path = circlePath.cgPath
        //change the fill color
        if (fill == true){
            circle.fillColor = color.cgColor
        }  
        else {
            circle.fillColor = UIColor.clear.cgColor
        }
        //you can change the stroke color
        circle.strokeColor = color.cgColor
        //you can change the line width
        circle.lineWidth = 3.0
        
        view.layer.addSublayer(circle)
    }
    
    func XY2RPhi(x: CGFloat, y: CGFloat) -> (r: CGFloat, phi: CGFloat){
        var r = CGFloat(0)
        var phi = CGFloat(0)
        
        r = sqrt(x*x + y*y)
        phi = atan2(-y,x)
        
        if (phi<0){
            phi = phi + 2*CGFloat.pi
        }

        return (r,phi)
    }
    
    func RPhi2XY(r: CGFloat, phi: CGFloat) -> (x: CGFloat, y: CGFloat){
        var x = CGFloat(0)
        var y = CGFloat(0)
        
        x = r*cos(phi)
        y = r*sin(phi + CGFloat.pi)
        
        return (x,y)
    }
    
    
    func scheduledTimerWithTimeInterval(){
        // Scheduling timer to Call the function "updateCounting" with the interval of 1 seconds
        timer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(SerialViewController.updateCounting), userInfo: nil, repeats: true)
    }
    
    @objc func updateCounting(){
        
        sendLength = Int(joyRvalue * 100 / (baseWidth/2 - buttonSize))
        sendAngle = Int(joyPhivalue * 180 / 2 / CGFloat.pi)
        
        sendButton = 0 + 1*AState + 2*BState + 4*CState + 8*DState + 16*switchState
        
        aliveCounter+=1
        
        infoText.text! = String(format: "x:%.2f, y:%.2f\nx:%.2f, y:%.2f\nr:%.2f, phi:%.2f\nr:%d, phi:%d\nbutton:%d\nAC:%d", location.x,location.y,joyXvalue, joyYvalue,joyRvalue, joyPhivalue, sendLength, sendAngle, sendButton, aliveCounter%16)
        
        if serial.isReady {
            serial.sendBytesToDevice([0xFF,UInt8(sendLength), UInt8(sendAngle), UInt8(sendButton), UInt8(aliveCounter%16)])
            //serial.sendMessageToDevice(String(format: "%d%d", sendLength, sendAngle))
        }
        
    }
    
    @objc func keyboardWillShow(_ notification: Notification) {
        // animate the text field to stay above the keyboard
        let info = (notification as NSNotification).userInfo!
        let value = info[UIResponder.keyboardFrameEndUserInfoKey] as! NSValue
        let keyboardFrame = value.cgRectValue
        
        //TODO: Not animating properly
        UIView.animate(withDuration: 1, delay: 0, options: UIView.AnimationOptions(), animations: { () -> Void in
            self.bottomConstraint.constant = keyboardFrame.size.height
            }, completion: { Bool -> Void in
            self.textViewScrollToBottom()
        })
    }
    
    @objc func keyboardWillHide(_ notification: Notification) {
        // bring the text field back down..
        UIView.animate(withDuration: 1, delay: 0, options: UIView.AnimationOptions(), animations: { () -> Void in
            self.bottomConstraint.constant = 0
        }, completion: nil)

    }
    
    @objc func reloadView() {
        // in case we're the visible view again
        serial.delegate = self
        
        if serial.isReady {
            navItem.title = serial.connectedPeripheral!.name
            barButton.title = "Disconnect"
            barButton.tintColor = UIColor.red
            barButton.isEnabled = true
        } else if serial.centralManager.state == .poweredOn {
            navItem.title = "Revolution Robotics"
            barButton.title = "Connect"
            barButton.tintColor = view.tintColor
            barButton.isEnabled = true
        } else {
            navItem.title = "Revolution Robotics"
            barButton.title = "Connect"
            barButton.tintColor = view.tintColor
            barButton.isEnabled = false
        }
    }
    
    func textViewScrollToBottom() {
        let range = NSMakeRange(NSString(string: mainTextView.text).length - 1, 1)
        mainTextView.scrollRangeToVisible(range)
    }
    

//MARK: BluetoothSerialDelegate
    
    func serialDidReceiveString(_ message: String) {
        // add the received text to the textView, optionally with a line break at the end
        mainTextView.text! += message
        let pref = UserDefaults.standard.integer(forKey: ReceivedMessageOptionKey)
        if pref == ReceivedMessageOption.newline.rawValue { mainTextView.text! += "\n" }
        textViewScrollToBottom()
    }
    
    func serialDidDisconnect(_ peripheral: CBPeripheral, error: NSError?) {
        reloadView()
        dismissKeyboard()
        let hud = MBProgressHUD.showAdded(to: view, animated: true)
        hud?.mode = MBProgressHUDMode.text
        hud?.labelText = "Disconnected"
        hud?.hide(true, afterDelay: 1.0)
    }
    
    func serialDidChangeState() {
        reloadView()
        if serial.centralManager.state != .poweredOn {
            dismissKeyboard()
            let hud = MBProgressHUD.showAdded(to: view, animated: true)
            hud?.mode = MBProgressHUDMode.text
            hud?.labelText = "Bluetooth turned off"
            hud?.hide(true, afterDelay: 1.0)
        }
    }
    
    
//MARK: UITextFieldDelegate
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if !serial.isReady {
            let alert = UIAlertController(title: "Not connected", message: "What am I supposed to send this to?", preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "Dismiss", style: UIAlertAction.Style.default, handler: { action -> Void in self.dismiss(animated: true, completion: nil) }))
            present(alert, animated: true, completion: nil)
            messageField.resignFirstResponder()
            return true
        }
        
        // send the message to the bluetooth device
        // but fist, add optionally a line break or carriage return (or both) to the message
        let pref = UserDefaults.standard.integer(forKey: MessageOptionKey)
        var msg = messageField.text!
        switch pref {
        case MessageOption.newline.rawValue:
            msg += "\n"
        case MessageOption.carriageReturn.rawValue:
            msg += "\r"
        case MessageOption.carriageReturnAndNewline.rawValue:
            msg += "\r\n"
        default:
            msg += ""
        }
        
        // send the message and clear the textfield
        serial.sendMessageToDevice(msg)
        messageField.text = ""
        return true
    }
    
    @objc func dismissKeyboard() {
        messageField.resignFirstResponder()
    }
    
    
//MARK: IBActions

    @IBAction func barButtonPressed(_ sender: AnyObject) {
        if serial.connectedPeripheral == nil {
            performSegue(withIdentifier: "ShowScanner", sender: self)
        } else {
            serial.disconnect()
            reloadView()
        }
    }
}
